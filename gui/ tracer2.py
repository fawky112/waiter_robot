#!/usr/bin/env python3
"""
Al Fanan — Full Communication Tracer
Shows BOTH WebSocket messages AND ROS2 topic messages in one terminal.
Run on the Pi (where ROS2 is available):
    source /opt/ros/humble/setup.bash
    python3 tracer2.py
"""

import asyncio
import threading
import websockets
import json
from datetime import datetime

PI_IP   = '172.20.10.6'
WS_PORT = 8765

# ── Colours ─────────────────────────────────────────────────
R  = '\033[91m'
G  = '\033[92m'
Y  = '\033[93m'
B  = '\033[94m'
M  = '\033[95m'
C  = '\033[96m'
W  = '\033[97m'
DIM= '\033[2m'
BLD= '\033[1m'
RST= '\033[0m'

# ── Meanings ─────────────────────────────────────────────────
TABLE_STATES = {
    0:  'IDLE',
    11: 'Home → Table 1 (take order)',
    12: 'Table 1 → Home (after order)',
    13: 'Home → Kitchen',
    14: 'Kitchen → Home (with food)',
    15: 'Home → Table 1 (deliver food)',
    16: 'Table 1 → Home (final)',
    17: 'Mission complete',
    18: 'Arrived at destination ✅',
    19: 'Obstacle detected ⚠️',
    20: 'Stepper UP',
    21: 'Stepper DOWN',
    22: 'Obstacle avoiding',
    30: 'Fault / E-Stop 🔴',
    41: 'Home → Table 2 (take order)',
    42: 'Table 2 → Home (after order)',
    45: 'Home → Table 2 (deliver food)',
    46: 'Table 2 → Home (final)',
}

SCREEN_VALS = {
    5: 'Payment done → robot go to kitchen',
    6: 'Customer done → robot go home final',
    7: 'Re-order payment done → robot reset',
}

GUI_TOPIC_VALS = {
    4: 'Kitchen Bell Table 1',
    5: 'Table 1 Available (camera unlock)',
    7: 'Kitchen Bell Table 2',
    8: 'Table 2 Available (camera unlock)',
}

MANU_VALS = {
    1: 'Forward',   2: 'Backward',
    3: 'Turn Right',4: 'Turn Left',
    5: 'Stepper Up',6: 'Stepper Down',
    7: 'Stop',      8: 'Enter Manual',
    9: 'Exit Manual',
}

DETECTION_VALS = {
    1: 'Customer at Table 1',
    2: 'Customer at Table 2',
}

MANU_STR = {
    'M:1':'Forward','M:2':'Backward','M:3':'Turn Right','M:4':'Turn Left',
    'M:5':'Stepper Up','M:6':'Stepper Down','M:7':'Stop',
    'M:8':'Enter Manual','M:9':'Exit Manual',
}

# ── Shared print lock ────────────────────────────────────────
_lock  = threading.Lock()
_count = 0

def log(source_col, source_tag, msg_col, message):
    global _count
    with _lock:
        _count += 1
        ts = datetime.now().strftime('%H:%M:%S.%f')[:-3]
        print(f'{DIM}{_count:>4}  {ts}{RST}  '
              f'{source_col}{BLD}{source_tag:<14}{RST}  '
              f'{msg_col}{message}{RST}')

# ── Decode WebSocket message ─────────────────────────────────
def decode_ws(raw: str) -> tuple:
    if raw.startswith('M:'):
        meaning = MANU_STR.get(raw, 'unknown manual cmd')
        return M, f'{raw}  →  {meaning}'

    if raw == 'paid':
        return G, 'Payment page → customer confirmed payment'

    if raw == '3':
        return C, '"3" → kiosk_bridge trigger → kiosk plays vid1/vid6'

    try:
        data = json.loads(raw)
        if 'order_items' in data:
            table = data.get('table','?')
            total = data.get('total',0)
            items = data.get('order_items',[])
            names = ', '.join(f"{i.get('name','?')}×{i.get('qty',1)}" for i in items)
            return Y, f'ORDER  table={table}  total=${total:.2f}  [{names}]'
        return Y, f'JSON: {raw[:100]}'
    except Exception:
        pass

    try:
        n = int(raw)
    except ValueError:
        return DIM, f'unknown: {raw!r}'

    if n in TABLE_STATES:
        col = (R if n in (19,30) else G if n==18 else
               B if n in (11,12,13,14,15,16,41,42,45,46) else
               Y if n==17 else W)
        return col, f'table state {n:>2}  →  {TABLE_STATES[n]}'

    if n in SCREEN_VALS:
        return C, f'screen signal {n}  →  {SCREEN_VALS[n]}'

    if n in (40,70,50,80):
        labels = {40:'Bell T1 (raw)',70:'Bell T2 (raw)',
                  50:'Available T1 (raw)',80:'Available T2 (raw)'}
        return Y, f'gui cmd {n}  →  {labels[n]}'

    if n in (3,6,7):
        labels = {3:'vid1 trigger',6:'customer done',7:'re-order done'}
        return DIM, f'kiosk signal {n}  →  {labels[n]}'

    return DIM, f'int {n}'


# ── WebSocket tracer ─────────────────────────────────────────
async def ws_tracer():
    uri = f'ws://{PI_IP}:{WS_PORT}'
    while True:
        try:
            async with websockets.connect(uri) as ws:
                log(G, '[WS]', G, f'Connected to {uri}')
                async for message in ws:
                    col, text = decode_ws(str(message).strip())
                    log(C, '[WS ←→]', col, text)
        except Exception as e:
            log(R, '[WS]', R, f'Disconnected: {e} — retrying in 3s')
            await asyncio.sleep(3)


# ── ROS2 tracer ──────────────────────────────────────────────
def ros2_tracer():
    try:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import Int32
    except ImportError:
        log(R, '[ROS2]', R, 'rclpy not found — ROS2 tracing disabled')
        return

    rclpy.init()

    class Tracer(Node):
        def __init__(self):
            super().__init__('alfanan_tracer')

            self.create_subscription(Int32, '/table',
                lambda m: self._cb('/table', m.data, TABLE_STATES,
                                   lambda n: R if n in (19,30) else
                                             G if n==18 else
                                             B if n in (11,12,13,14,15,16,41,42,45,46) else
                                             Y if n==17 else W), 10)

            self.create_subscription(Int32, '/screen',
                lambda m: self._cb('/screen', m.data, SCREEN_VALS, lambda _: C), 10)

            self.create_subscription(Int32, '/gui_topic',
                lambda m: self._cb('/gui_topic', m.data, GUI_TOPIC_VALS, lambda _: Y), 10)

            self.create_subscription(Int32, '/manu',
                lambda m: self._cb('/manu', m.data, MANU_VALS, lambda _: M), 10)

            self.create_subscription(Int32, '/detection',
                lambda m: self._cb('/detection', m.data, DETECTION_VALS, lambda _: G), 10)

        def _cb(self, topic, val, lookup, color_fn):
            meaning = lookup.get(val, f'unknown value {val}')
            col     = color_fn(val)
            tag     = f'[ROS2 {topic}]'
            log(B, tag, col, f'{val}  →  {meaning}')

    node = Tracer()
    log(B, '[ROS2]', B, 'Subscribed to /table /screen /gui_topic /manu /detection')
    try:
        rclpy.spin(node)
    except Exception:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


# ── Main ─────────────────────────────────────────────────────
if __name__ == '__main__':
    print(f'\n{C}{BLD}Al Fanan — Full Communication Tracer{RST}')
    print(f'{DIM}WebSocket + ROS2 topics in one view{RST}')
    print(f'{DIM}{"─"*65}{RST}')
    print(f'{DIM}  #    time          source          message{RST}')
    print(f'{DIM}{"─"*65}{RST}\n')

    # ROS2 in background thread
    ros_thread = threading.Thread(target=ros2_tracer, daemon=True)
    ros_thread.start()

    # WebSocket in main async loop
    try:
        asyncio.run(ws_tracer())
    except KeyboardInterrupt:
        print(f'\n{DIM}Tracer stopped. {_count} messages logged.{RST}')
