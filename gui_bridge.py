#!/usr/bin/env python3
"""
Al Fanan Restaurant — GUI Bridge  (ROS2 ↔ WebSocket)
═══════════════════════════════════════════════════════

Runs on the laptop. Connects to the Pi's server.py over WebSocket
and translates kitchen-GUI button presses into ROS2 publishes.

MESSAGES FROM GUI → BRIDGE → ROS2
──────────────────────────────────
  M:1 … M:9   Manual mode commands     → /manu  (1=Fwd 2=Back 3=R 4=L
                                                  5=StepUp 6=StepDn 7=Stop
                                                  8=Enter 9=Exit)

  40           Kitchen Bell Table 1    → /gui_topic = 4
  70           Kitchen Bell Table 2    → /gui_topic = 7

  50           Table 1 Available       → /gui_topic = 5
  80           Table 2 Available       → /gui_topic = 8

  5,6,7        Kiosk screen signals    → ignored here (server already
                                         relays them to all clients)

NOTE: /table is NOT subscribed here — kiosk_bridge.py already
forwards every /table state to the WebSocket. Subscribing here too
would cause every state to arrive at the kitchen GUI TWICE, breaking
the pendingHomeArrival flag and making the bell open then close.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import websocket
import threading
import time

PI_IP = '172.20.10.6'   # ← change when network changes


class GUIBridge(Node):
    def __init__(self):
        super().__init__('gui_bridge')

        self.gui_pub  = self.create_publisher(Int32, 'gui_topic', 10)
        self.manu_pub = self.create_publisher(Int32, 'manu',      10)

        # /table NOT subscribed — kiosk_bridge handles forwarding

        self.ws_url = f'ws://{PI_IP}:8765'
        threading.Thread(target=self._ws_loop, daemon=True).start()
        self.get_logger().info(f'🖥️  GUI Bridge online → {self.ws_url}')

    # ── WebSocket listener ───────────────────────────────────────────
    def _ws_loop(self):
        while rclpy.ok():
            try:
                ws = websocket.create_connection(self.ws_url)
                self.get_logger().info('✅ Connected to server')

                while True:
                    raw = ws.recv()
                    if raw is None:
                        break
                    data = raw.strip()
                    if not data:
                        continue

                    self.get_logger().info(f'[WS in] {data!r}')
                    self._handle(data)

            except Exception as e:
                self.get_logger().warn(f'WS error: {e} — reconnecting in 2 s')
                time.sleep(2)

    # ── Message router ───────────────────────────────────────────────
    def _handle(self, data: str):

        # ── M:N  manual mode commands ────────────────────────────────
        # GUI sends "M:1", "M:2" … "M:9" so they can't clash with
        # integer robot states. Must be checked BEFORE int() parse.
        if data.startswith('M:'):
            try:
                n = int(data[2:])
                self.get_logger().info(f'🕹️  Manual M:{n} → /manu={n}')
                self._pub(self.manu_pub, n)
            except ValueError:
                self.get_logger().warn(f'Bad manual cmd: {data!r}')
            return

        # ── Parse integer ────────────────────────────────────────────
        try:
            cmd = int(data)
        except ValueError:
            # JSON order data or unknown string — already relayed by
            # server to all clients, nothing to do here.
            return

        # ── Kitchen Bell Table 1  (40 → /gui_topic = 4) ─────────────
        if cmd == 40:
            self.get_logger().info('🔔 Bell T1 → /gui_topic=4')
            self._pub(self.gui_pub, 4)

        # ── Kitchen Bell Table 2  (70 → /gui_topic = 7) ─────────────
        elif cmd == 70:
            self.get_logger().info('🔔 Bell T2 → /gui_topic=7')
            self._pub(self.gui_pub, 7)

        # ── Table 1 Available  (50 → /gui_topic = 5) ────────────────
        elif cmd == 50:
            self.get_logger().info('✅ Table 1 Available → /gui_topic=5')
            self._pub(self.gui_pub, 5)

        # ── Table 2 Available  (80 → /gui_topic = 8) ────────────────
        elif cmd == 80:
            self.get_logger().info('✅ Table 2 Available → /gui_topic=8')
            self._pub(self.gui_pub, 8)

        # ── Kiosk screen signals (5, 6, 7) ──────────────────────────
        # Already relayed by server.py to all WebSocket clients.
        # No ROS2 publish needed from the GUI bridge side.
        elif cmd in (5, 6, 7):
            self.get_logger().info(f'ℹ️  Kiosk signal {cmd} — relay handled by server')

        # ── Raw integer 1-9 fallback ─────────────────────────────────
        # Should not normally arrive (manual mode uses M:N strings),
        # but kept as a safety net.
        elif 1 <= cmd <= 9:
            self.get_logger().warn(f'⚠️  Raw int {cmd} on /manu — use M:{cmd} instead')
            self._pub(self.manu_pub, cmd)

    # ── Publish 3× for reliability ───────────────────────────────────
    def _pub(self, publisher, value, repeats=3, gap=0.05):
        msg = Int32()
        msg.data = int(value)
        for _ in range(repeats):
            publisher.publish(msg)
            time.sleep(gap)


def main():
    rclpy.init()
    node = GUIBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
