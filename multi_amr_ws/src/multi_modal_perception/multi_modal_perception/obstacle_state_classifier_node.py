#!/usr/bin/env python3

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


VALID_SEMANTIC_CLASSES = {'HUMAN', 'CART', 'FORKLIFT'}


class ObstacleStateClassifierNode(Node):


    def __init__(self):
        super().__init__('obstacle_state_classifier_node')

        self.declare_parameter('input_topic', '/fusion/semantic_objects')
        self.declare_parameter('classified_topic', '/perception/classified_objects')
        self.declare_parameter('marker_topic', '/perception/classified_object_markers')

        self.declare_parameter('visual_max_distance', 8.0)
        self.declare_parameter('visual_show_static', True)
        self.declare_parameter('visual_max_labels', 6)

        self.input_topic = self.get_parameter('input_topic').value
        self.classified_topic = self.get_parameter('classified_topic').value
        self.marker_topic = self.get_parameter('marker_topic').value

        self.visual_max_distance = float(self.get_parameter('visual_max_distance').value)
        self.visual_show_static = bool(self.get_parameter('visual_show_static').value)
        self.visual_max_labels = int(self.get_parameter('visual_max_labels').value)

        self.subscription = self.create_subscription(
            String,
            self.input_topic,
            self.callback,
            10
        )

        self.classified_pub = self.create_publisher(
            String,
            self.classified_topic,
            10
        )

        self.marker_pub = self.create_publisher(
            MarkerArray,
            self.marker_topic,
            10
        )

        self.get_logger().info(
            f'Simple object classifier started | classes=HUMAN,CART,FORKLIFT,STATIC | '
            f'input={self.input_topic}'
        )

    def callback(self, msg):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn('Invalid associated object JSON received.')
            return

        header = data.get('header', {})
        output = []

        for obj in data.get('objects', []):
            object_id = int(obj.get('object_id', -1))
            fusion_source = obj.get('fusion_source', 'lidar_only')
            raw_label = str(obj.get('class_label', 'STATIC')).upper()
            class_confidence = float(obj.get('class_confidence', 0.0))

            if fusion_source in {'camera_lidar_associated', 'semantic_persistent'} and raw_label in VALID_SEMANTIC_CLASSES:
                final_label = raw_label
                object_type = 'semantic_dynamic_object'
                motion_state = 'dynamic'
                confidence = class_confidence
                if fusion_source == 'semantic_persistent':
                    object_type = 'semantic_persistent_object'
            else:
                final_label = 'STATIC'
                object_type = 'static_structure'
                motion_state = 'static'
                confidence = 1.0

            size_values = obj.get('size', [0.0, 0.0])
            size_x = float(size_values[0]) if len(size_values) > 0 else 0.0
            size_y = float(size_values[1]) if len(size_values) > 1 else 0.0

            output.append({
                'object_id': object_id,
                'class_label': final_label,
                'object_type': object_type,
                'classification_confidence': round(confidence, 3),
                'motion_state': motion_state,
                'distance': round(float(obj.get('distance', 0.0)), 3),
                'size': [round(size_x, 3), round(size_y, 3)],
                'num_points': int(obj.get('num_points', 0)),
                'position_lidar': obj.get('position_lidar', [0.0, 0.0]),
                'projected_pixel': obj.get('projected_pixel', []),
                'associated_bbox': obj.get('associated_bbox', []),
                'fusion_source': fusion_source,
                'semantic_object': final_label in VALID_SEMANTIC_CLASSES,
                'semantic_persistence': bool(obj.get('semantic_persistence', False)),
                'semantic_memory_id': int(obj.get('semantic_memory_id', -1)),
                'semantic_age_sec': round(float(obj.get('semantic_age_sec', 0.0)), 3),
            })

        out = String()
        out.data = json.dumps({
            'header': header,
            'objects': output,
        })
        self.classified_pub.publish(out)
        self.publish_markers(header, output)

    def should_visualize(self, obj):
        if obj['distance'] > self.visual_max_distance:
            return False
        if obj['class_label'] == 'STATIC' and not self.visual_show_static:
            return False
        return True

    def publish_markers(self, header, objects):
        marker_array = MarkerArray()
        frame_id = header.get('frame_id', 'laser')

        delete_marker = Marker()
        delete_marker.header.frame_id = frame_id
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        visible_objects = [obj for obj in objects if self.should_visualize(obj)]

        # Show semantic objects first, then nearby static structures.
        visible_objects.sort(
            key=lambda obj: (
                0 if obj['class_label'] in VALID_SEMANTIC_CLASSES else 1,
                obj['distance']
            )
        )

        for i, obj in enumerate(visible_objects[: self.visual_max_labels]):
            x, y = obj.get('position_lidar', [0.0, 0.0])
            label = obj['class_label']

            marker = Marker()
            marker.header.frame_id = frame_id
            marker.ns = 'classified_object_labels'
            marker.id = i
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD

            marker.pose.position.x = float(x)
            marker.pose.position.y = float(y)
            marker.pose.position.z = 1.0
            marker.pose.orientation.w = 1.0

            marker.scale.z = 0.25
            marker.color.a = 1.0

            if label == 'HUMAN':
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            elif label == 'FORKLIFT':
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 0.6
            elif label == 'CART':
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            else:
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0

            if label in VALID_SEMANTIC_CLASSES:
                confidence_percent = int(round(float(obj['classification_confidence']) * 100.0))
                marker.text = f'{label} {confidence_percent}%'
            else:
                marker.text = 'STATIC'

            marker.lifetime.nanosec = 300000000
            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleStateClassifierNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()