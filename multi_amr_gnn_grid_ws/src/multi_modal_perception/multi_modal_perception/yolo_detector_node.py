#!/usr/bin/env python3

import json
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None


class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector_node')

        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('model_name', 'yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.35)
        self.declare_parameter('publish_annotated_image', True)

        self.image_topic = self.get_parameter('image_topic').value
        self.model_name = self.get_parameter('model_name').value
        self.confidence_threshold = float(
            self.get_parameter('confidence_threshold').value
        )
        self.publish_annotated_image = bool(
            self.get_parameter('publish_annotated_image').value
        )

        if YOLO is None:
            self.get_logger().error(
                'Ultralytics is not installed. Activate ~/yolo_venv and set PYTHONPATH.'
            )
            raise RuntimeError('Missing ultralytics package')

        self.bridge = CvBridge()
        self.model = YOLO(self.model_name)

        self.detection_pub = self.create_publisher(String, '/yolo/detections', 10)
        self.annotated_image_pub = self.create_publisher(Image, '/yolo/annotated_image', 10)

        self.last_time = time.time()
        self.frame_count = 0

        self.subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )

        self.get_logger().info(
            f'YOLO detector started | topic={self.image_topic} | model={self.model_name}'
        )

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        results = self.model.predict(
            source=frame,
            conf=self.confidence_threshold,
            verbose=False
        )

        annotated = frame.copy()
        detections = []

        if results:
            result = results[0]

            if result.boxes is not None:
                for box in result.boxes:
                    cls_id = int(box.cls[0])
                    conf = float(box.conf[0])
                    label = self.model.names.get(cls_id, str(cls_id))

                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)

                    detections.append({
                        'class_id': cls_id,
                        'label': label,
                        'confidence': conf,
                        'bbox': [int(x1), int(y1), int(x2), int(y2)],
                        'stamp_sec': int(msg.header.stamp.sec),
                        'stamp_nanosec': int(msg.header.stamp.nanosec),
                        'frame_id': msg.header.frame_id,
                    })

                    cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)

                    confidence_percent = int(round(conf * 100.0))
                    text = f'{label} {confidence_percent}%'
                    cv2.putText(
                        annotated,
                        text,
                        (x1, max(y1 - 8, 20)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.55,
                        (0, 255, 0),
                        2
                    )

        msg_out = String()
        msg_out.data = json.dumps({
            'header': {
                'stamp_sec': int(msg.header.stamp.sec),
                'stamp_nanosec': int(msg.header.stamp.nanosec),
                'frame_id': msg.header.frame_id,
            },
            'detections': detections,
        })
        self.detection_pub.publish(msg_out)

        if self.publish_annotated_image:
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            annotated_msg.header = msg.header
            self.annotated_image_pub.publish(annotated_msg)

        self.frame_count += 1
        now = time.time()

        if now - self.last_time >= 1.0:
            fps = self.frame_count / (now - self.last_time)
            self.get_logger().info(
                f'YOLO FPS: {fps:.2f} | detections: {len(detections)}'
            )
            self.frame_count = 0
            self.last_time = now


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()