#!/usr/bin/env python3
"""Save one frame from a ROS 2 image topic as a PNG."""

import argparse
from pathlib import Path

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class ImageCapture(Node):
    def __init__(self, topic: str, output: Path) -> None:
        super().__init__('simulation_image_capture')
        self.output = output
        self.bridge = CvBridge()
        self.captured = False
        self.subscription = self.create_subscription(Image, topic, self.callback, 10)

    def callback(self, message: Image) -> None:
        if self.captured:
            return
        image = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        self.output.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(self.output), image)
        self.captured = True
        self.get_logger().info(f'Saved {self.output}')


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('topic')
    parser.add_argument('output', type=Path)
    args = parser.parse_args()

    rclpy.init()
    capture = ImageCapture(args.topic, args.output)
    while rclpy.ok() and not capture.captured:
        rclpy.spin_once(capture, timeout_sec=1.0)
    capture.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
