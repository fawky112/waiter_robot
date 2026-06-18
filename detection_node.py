#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import cv2
from ultralytics import YOLO
import time


class PersonDetectionNode(Node):

    def __init__(self):
        super().__init__('person_detection_node')

        # Publisher
        self.publisher = self.create_publisher(
            Int32,
            'detection',
            10
        )

        # Subscriber to table topic
        self.table_sub = self.create_subscription(
            Int32,
            'table',
            self.table_callback,
            10
        )

        # Detection enabled only after receiving table=1
        self.table_trigger = False

        # Detection timer
        self.timer = self.create_timer(
            0.2,
            self.timer_callback
        )

        # Camera
        self.cap = cv2.VideoCapture(0)

        if not self.cap.isOpened():
            self.get_logger().error("Cannot open camera")
            return

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

        # YOLO model
        model_path = "/home/kratos/run4/weights/best.pt"
        self.model = YOLO(model_path)

        self.frame_count = 0
        self.frame_skip = 2

        self.last_output = 0
        self.last_print_time = time.time()

        self.get_logger().info("✅ Person Detection Node Started")
        self.get_logger().info(
            "📡 Waiting for /table = 1 to enable detection publishing"
        )

    # -------------------------------------------------
    # TABLE CALLBACK
    # -------------------------------------------------
    def table_callback(self, msg):

        # Enable detection publishing
        if msg.data == 1:

            self.table_trigger = True

            self.get_logger().info(
                "✅ Received table=1 -> Detection ENABLED"
            )

        # Disable detection publishing
        elif msg.data in [11, 41, 13]:

            self.table_trigger = False

            self.get_logger().info(
                f"🛑 Received table={msg.data} -> Detection DISABLED"
            )

    # -------------------------------------------------
    # DETECTION LOOP
    # -------------------------------------------------
    def timer_callback(self):

        ret, frame = self.cap.read()

        if not ret:
            return

        self.frame_count += 1

        frame_height = frame.shape[0]
        frame_width = frame.shape[1]
        mid_x = frame_width // 2

        output_value = 0

        if self.frame_count % self.frame_skip == 0:

            results = self.model(
                frame,
                imgsz=320,
                verbose=False
            )[0]

            if results.boxes is not None:

                for box in results.boxes:

                    cls_id = int(box.cls[0])
                    conf = float(box.conf[0])

                    if conf < 0.5:
                        continue

                    # Sitting person
                    if cls_id == 3:

                        x1, y1, x2, y2 = map(
                            int,
                            box.xyxy[0]
                        )

                        cx = (x1 + x2) // 2
                        cy = (y1 + y2) // 2

                        distance_from_bottom = (
                            frame_height - cy
                        )

                        if distance_from_bottom < frame_height * 0.30:

                            # -------------------------
                            # RIGHT
                            # -------------------------
                            if cx > mid_x:

                                output_value = 1

                                if self.table_trigger:

                                    msg = Int32()
                                    msg.data = 1

                                    self.publisher.publish(msg)

                            # -------------------------
                            # LEFT
                            # -------------------------
                            else:

                                output_value = 2

                                if self.table_trigger:

                                    msg = Int32()
                                    msg.data = 2

                                    self.publisher.publish(msg)

                    # Other classes
                    elif cls_id == 0:
                        output_value = 3

                    elif cls_id == 1:
                        output_value = 4

        # Status logging
        current_time = time.time()

        if (
            output_value != self.last_output
            or (current_time - self.last_print_time) > 2.0
        ):

            if output_value == 1:

                if self.table_trigger:
                    self.get_logger().info(
                        "🎯 RIGHT detected -> Publishing 1"
                    )
                else:
                    self.get_logger().info(
                        "🎯 RIGHT detected (Publishing Disabled)"
                    )

            elif output_value == 2:

                if self.table_trigger:
                    self.get_logger().info(
                        "🎯 LEFT detected -> Publishing 2"
                    )
                else:
                    self.get_logger().info(
                        "🎯 LEFT detected (Publishing Disabled)"
                    )

            elif output_value == 0:

                self.get_logger().info(
                    "🔍 Scanning..."
                )

            self.last_output = output_value
            self.last_print_time = current_time

    # -------------------------------------------------
    # CLEANUP
    # -------------------------------------------------
    def destroy_node(self):

        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()

        super().destroy_node()

    def __del__(self):

        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()


def main(args=None):

    rclpy.init(args=args)

    node = PersonDetectionNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        if 'node' in locals():
            node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()
