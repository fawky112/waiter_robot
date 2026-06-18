#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════════
Al Fanan Restaurant — Kiosk Bridge  (ROS2 ↔ WebSocket)
═══════════════════════════════════════════════════════════════════
Runs ON the Pi. Subscribes to ESP states on /table and forwards them
to the kiosk's WebSocket so the customer-facing screens (loading
overlays, vid1/2/3/4/5/6, menus) follow the robot's progress.
Listens for kiosk events (payment, customer-done, order-more) and
publishes them as ROS2 commands the ESP can act on.

INTEGER-ONLY PROTOCOL (no strings between bridge and kiosk).

  ─── ROS2 Topics ───────────────────────────────────────────────

    SUBSCRIBES:
      /table       Int32   ← ESP32 nav states
      /detection   Int32   ← camera / gesture detection

    PUBLISHES:
      /screen        Int32   → ESP32 sequence commands (5/6/7)
      /payment_done  Int32   → payment confirmation (value 3)

  ─── WebSocket (ws://PI_IP:8765) ───────────────────────────────

    From kiosk:
       5 = payment done, continue sequence
       6 = customer done, go home final
       7 = order more, stay at table
       (JSON order strings get relayed by server.py unchanged)

    To kiosk:
       "11".."46"  → ESP state numbers (drive the loading screens)
       "3"         → table arrival trigger (replaces "table:3")

  ─── ESP States on /table ──────────────────────────────────────

    Shared / generic:
       0  Idle               13 H→Kitchen          14 Kitchen→Home (food)
       17 COMPLETE           18 Arrived/Waiting    19 Obstacle
       20 Stepper UP         21 Stepper DOWN       22 Avoiding

    Table 1 mission:
       11 H→T1 (take order)        12 T1→Home (after order)
       15 H→T1 (delivering food)   16 T1→Home (final)

    Table 2 mission  (NEW — mirrors T1):
       41 H→T2 (take order)        42 T2→Home (after order)
       45 H→T2 (delivering food)   46 T2→Home (final)

  ⚡ Critical commands (5/6/7) are published 5 TIMES with 100 ms gaps
     so a single dropped Wi-Fi packet doesn't kill the sequence.
═══════════════════════════════════════════════════════════════════
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import websocket
import threading
import time
import json


class KioskBridge(Node):
    def __init__(self):
        super().__init__('kiosk_bridge')

        # ── Publishers → ROS2 ──────────────────────────
        self.screen_pub  = self.create_publisher(Int32, 'screen',       10)
        self.payment_pub = self.create_publisher(Int32, 'payment_done', 10)

        # ── Subscribers ← ROS2 ──────────────────────────
        self.table_sub = self.create_subscription(
            Int32, 'table', self.table_callback, 10)
        self.detect_sub = self.create_subscription(
            Int32, 'detection', self.detect_callback, 10)

        # ── Config ──────────────────────────────────────
        self.ip_address = '172.20.10.6'   # Pi's IP address
        self.ws_url     = f'ws://{self.ip_address}:8765'

        # ── Track robot context ─────────────────────────
        self.robot_heading_to_table = False   # True after state 11
        self.robot_delivering_food  = False   # True after state 15
        self.robot_returning_home   = False   # True after state 12/16
        self.robot_at_table_first   = False   # True when arrived first time

        # ── WebSocket listener thread ───────────────────
        self.thread = threading.Thread(
            target=self.websocket_listener, daemon=True)
        self.thread.start()

        self.get_logger().info(
            f'🚀 KioskBridge online (INTEGER PROTOCOL)\n'
            f'   sub  ← /table, /detection\n'
            f'   pub  → /screen (5=continue, 6=go_home, 7=order_more) [5x each]\n'
            f'   pub  → /payment_done [5x]\n'
            f'   ws   → {self.ws_url}'
        )

    # ─────────────────────────────────────────────────────
    #  PUBLISH HELPERS
    #     publish_5x() sends the same message 5 times
    #     with 100ms gaps to ensure ESP32 reception.
    # ─────────────────────────────────────────────────────

    def publish_5x(self, publisher, value: int):
        """Publish the same Int32 message 5 times, 100ms apart."""
        msg = Int32()
        msg.data = value
        for i in range(5):
            publisher.publish(msg)
            self.get_logger().info(f'  📤 Published {value} (attempt {i+1}/5)')
            time.sleep(0.1)

    def publish_1x(self, publisher, value: int):
        """Publish a single Int32 message (for non-critical)."""
        msg = Int32()
        msg.data = value
        publisher.publish(msg)
        self.get_logger().info(f'  📤 Published {value}')

    # ─────────────────────────────────────────────────────
    #  ROS2 CALLBACKS  (ROS → WebSocket)
    # ─────────────────────────────────────────────────────

    def table_callback(self, msg):
        """
        ESP publishes nav_state on /table (integers 11-22).
        Bridge forwards them to kiosk as strings.
        When robot arrives at table (state 18 + heading=True),
        also sends "3" to trigger VID1.
        """
        state = msg.data
        self.get_logger().info(
            f'[/table] state: {state} | '
            f'heading={self.robot_heading_to_table}, '
            f'delivering={self.robot_delivering_food}'
        )

        # ── Track robot context based on states ──────────
        # Numbers 11/12/15/16 belong to a Table 1 mission.
        # Numbers 41/42/45/46 belong to a Table 2 mission and trigger
        # the SAME flow on the kiosk side — same loading screens,
        # same videos. The kiosk decides which table number to put in
        # the order JSON; the bridge doesn't need to track that.

        if state == 11 or state == 41:
            # Robot heading to table (first visit)
            self.robot_heading_to_table = True
            self.robot_at_table_first   = False
            self.robot_delivering_food  = False
            self.robot_returning_home   = False

        elif state == 12 or state == 42:
            # Robot returning home after order
            self.robot_heading_to_table = False
            self.robot_returning_home   = True

        elif state == 15 or state == 45:
            # Robot delivering food to table
            self.robot_delivering_food  = True
            self.robot_heading_to_table = False

        elif state == 16 or state == 46:
            # Robot returning home final (after delivery)
            self.robot_returning_home   = True
            self.robot_delivering_food  = False

        elif state == 17:
            # COMPLETE — reset all context (shared by T1 and T2)
            self.robot_heading_to_table = False
            self.robot_delivering_food  = False
            self.robot_returning_home   = False
            self.robot_at_table_first   = False

        # ── State 18 = Robot ARRIVED at destination ──────
        if state == 18:
            if self.robot_heading_to_table and not self.robot_at_table_first:
                # First arrival at table → send "3" to trigger VID1
                self.robot_at_table_first = True
                self.get_logger().info('📍 Arrived at table (first) → sending 3')
                self.send_to_server('3')

            elif self.robot_delivering_food:
                # Arrived at table with food
                self.get_logger().info('📍 Arrived at table with food')
                self.robot_delivering_food = False

            elif self.robot_returning_home:
                # Arrived at home
                self.get_logger().info('📍 Arrived at home')
                self.robot_returning_home = False

        # ── Always forward state to kiosk for loading screens ──
        self.send_to_server(str(state))

    def detect_callback(self, msg):
        """Camera / gesture detection signal."""
        self.get_logger().info(f'[/detection] value: {msg.data}')
        # Forward detection events to kiosk if needed
        if msg.data == 3:
            self.send_to_server('detection:3')

    # ─────────────────────────────────────────────────────
    #  WEBSOCKET → ROS2  (kiosk commands)
    #  Receives integers from kiosk:
    #    5 = payment done, continue sequence
    #    6 = customer done, go home final
    #    7 = order more, stay at table
    #  Also receives JSON order data for kitchen
    # ─────────────────────────────────────────────────────

    def websocket_listener(self):
        """
        Listens to WebSocket for kiosk messages.
        Pure relay except for known integers → publishes to ROS2.
        Critical commands (5,6,7) are published 5 TIMES.
        """
        while rclpy.ok():
            try:
                ws = websocket.create_connection(self.ws_url)
                self.get_logger().info('🔗 WebSocket connected to server')

                while True:
                    data = ws.recv()
                    self.get_logger().info(f'[WS RX] "{data}"')

                    # Try to parse as integer
                    try:
                        cmd = int(data)
                        
                        if cmd == 5:
                            # Payment done → continue sequence (5x)
                            self.get_logger().info(
                                '💰 payment_done (5) → /screen=5, /payment_done=3')
                            self.publish_5x(self.screen_pub, 5)
                            self.publish_5x(self.payment_pub, 3)

                        elif cmd == 6:
                            # Customer done → go home final (5x)
                            self.get_logger().info(
                                '👋 customer_done (6) → /screen=6')
                            self.publish_5x(self.screen_pub, 6)

                        elif cmd == 7:
                            # Order more → stay at table, continue (5x)
                            self.get_logger().info(
                                '🍽️ order_more (7) → /screen=7')
                            self.publish_5x(self.screen_pub, 7)

                        else:
                            # Other integers (like ESP states echoed back) — ignore
                            self.get_logger().info(
                                f'[WS] Unhandled integer: {cmd}')

                    except ValueError:
                        # Not an integer — could be JSON order or other message
                        self.get_logger().info(
                            f'[WS] Non-integer message (relayed): {data[:100]}')

            except Exception as e:
                self.get_logger().warn(f'WS error: {e} — reconnecting in 2s...')
                time.sleep(2)

    # ─────────────────────────────────────────────────────
    #  HELPERS
    # ─────────────────────────────────────────────────────

    def send_to_server(self, message: str):
        """Send a message to the WebSocket server."""
        try:
            ws = websocket.create_connection(self.ws_url, timeout=3)
            ws.send(message)
            ws.close()
        except Exception as e:
            self.get_logger().error(f'Send failed: {e}')


# ─────────────────────────────────────────────────────────
#  ENTRY POINT
# ─────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = KioskBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
