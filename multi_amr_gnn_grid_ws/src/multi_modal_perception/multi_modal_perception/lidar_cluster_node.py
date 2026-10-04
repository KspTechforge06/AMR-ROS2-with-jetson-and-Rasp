#!/usr/bin/env python3

import json
import math
from collections import deque

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


class LidarClusterNode(Node):
    def __init__(self):
        super().__init__('lidar_cluster_node')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cluster_topic', '/lidar/clusters')
        self.declare_parameter('marker_topic', '/lidar/cluster_markers')
        self.declare_parameter('min_range', 0.15)
        self.declare_parameter('max_range', 8.0)
        self.declare_parameter('cluster_distance', 0.28)
        self.declare_parameter('min_cluster_points', 4)
        self.declare_parameter('max_cluster_points', 120)
        self.declare_parameter('max_cluster_width', 1.40)
        self.declare_parameter('max_cluster_depth', 1.40)
        self.declare_parameter('max_cluster_area', 1.60)
        self.declare_parameter('max_cluster_aspect_ratio', 4.00)

        self.scan_topic = self.get_parameter('scan_topic').value
        self.cluster_topic = self.get_parameter('cluster_topic').value
        self.marker_topic = self.get_parameter('marker_topic').value
        self.min_range = float(self.get_parameter('min_range').value)
        self.max_range = float(self.get_parameter('max_range').value)
        self.cluster_distance = float(self.get_parameter('cluster_distance').value)
        self.min_cluster_points = int(self.get_parameter('min_cluster_points').value)
        self.max_cluster_points = int(self.get_parameter('max_cluster_points').value)
        self.max_cluster_width = float(self.get_parameter('max_cluster_width').value)
        self.max_cluster_depth = float(self.get_parameter('max_cluster_depth').value)
        self.max_cluster_area = float(self.get_parameter('max_cluster_area').value)
        self.max_cluster_aspect_ratio = float(
            self.get_parameter('max_cluster_aspect_ratio').value
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            10
        )

        self.cluster_pub = self.create_publisher(
            String,
            self.cluster_topic,
            10
        )

        self.marker_pub = self.create_publisher(
            MarkerArray,
            self.marker_topic,
            10
        )

        self.get_logger().info(
            f'LiDAR cluster node started | scan={self.scan_topic} | max_size={self.max_cluster_width:.2f}x{self.max_cluster_depth:.2f}m | max_aspect={self.max_cluster_aspect_ratio:.2f}'
        )

    def scan_callback(self, msg):
        points = []
        angle = msg.angle_min

        for r in msg.ranges:
            if math.isfinite(r) and self.min_range <= r <= self.max_range:
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                points.append((x, y))

            angle += msg.angle_increment

        clusters = self.euclidean_cluster(points)
        cluster_outputs = []

        for cluster_id, cluster in enumerate(clusters):
            xs = [p[0] for p in cluster]
            ys = [p[1] for p in cluster]

            min_x = min(xs)
            max_x = max(xs)
            min_y = min(ys)
            max_y = max(ys)

            cx = sum(xs) / len(xs)
            cy = sum(ys) / len(ys)

            width = max_x - min_x
            depth = max_y - min_y
            area = width * depth
            aspect_ratio = max(width, depth) / max(min(width, depth), 0.01)
            distance = math.sqrt(cx * cx + cy * cy)

            # Reject large rack/wall/aisle structures.
            # These are static warehouse geometry, not objects of interest.
            if (
                width > self.max_cluster_width
                or depth > self.max_cluster_depth
                or area > self.max_cluster_area
            ):
                continue

            # Reject long thin rack/aisle fragments.
            # Humans, carts, and forklifts are usually compact objects.
            # Rack edges often appear as long thin LiDAR clusters.
            if aspect_ratio > self.max_cluster_aspect_ratio:
                continue

            cluster_outputs.append({
                'id': int(cluster_id),
                'num_points': int(len(cluster)),
                'centroid': [float(cx), float(cy)],
                'bbox': [float(min_x), float(min_y), float(max_x), float(max_y)],
                'size': [float(width), float(depth)],
                'aspect_ratio': float(aspect_ratio),
                'distance': float(distance),
            })

        self.publish_clusters(msg, cluster_outputs)
        self.publish_markers(msg, cluster_outputs)

        self.get_logger().info(
            f'LiDAR clusters: {len(cluster_outputs)} | raw points: {len(points)}',
            throttle_duration_sec=1.0
        )

    def euclidean_cluster(self, points):
        if not points:
            return []

        visited = [False] * len(points)
        clusters = []

        for i in range(len(points)):
            if visited[i]:
                continue

            queue = deque([i])
            visited[i] = True
            cluster_indices = []

            while queue:
                idx = queue.popleft()
                cluster_indices.append(idx)

                px, py = points[idx]

                for j, (qx, qy) in enumerate(points):
                    if visited[j]:
                        continue

                    if math.hypot(px - qx, py - qy) <= self.cluster_distance:
                        visited[j] = True
                        queue.append(j)

            if self.min_cluster_points <= len(cluster_indices) <= self.max_cluster_points:
                clusters.append([points[index] for index in cluster_indices])

        return clusters

    def publish_clusters(self, scan_msg, clusters):
        out = String()
        out.data = json.dumps({
            'header': {
                'stamp_sec': int(scan_msg.header.stamp.sec),
                'stamp_nanosec': int(scan_msg.header.stamp.nanosec),
                'frame_id': scan_msg.header.frame_id,
            },
            'clusters': clusters,
        })

        self.cluster_pub.publish(out)

    def publish_markers(self, scan_msg, clusters):
        marker_array = MarkerArray()

        delete_marker = Marker()
        delete_marker.header.stamp = scan_msg.header.stamp
        delete_marker.header.frame_id = scan_msg.header.frame_id
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        for cluster in clusters:
            marker = Marker()
            marker.header.stamp = scan_msg.header.stamp
            marker.header.frame_id = scan_msg.header.frame_id
            marker.ns = 'lidar_clusters'
            marker.id = int(cluster['id'])
            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            cx, cy = cluster['centroid']
            width, depth = cluster['size']

            marker.pose.position.x = float(cx)
            marker.pose.position.y = float(cy)
            marker.pose.position.z = 0.25
            marker.pose.orientation.w = 1.0

            marker.scale.x = max(float(width), 0.08)
            marker.scale.y = max(float(depth), 0.08)
            marker.scale.z = 0.50

            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.65

            marker.lifetime.nanosec = 300000000
            marker_array.markers.append(marker)

            text = Marker()
            text.header.stamp = scan_msg.header.stamp
            text.header.frame_id = scan_msg.header.frame_id
            text.ns = 'lidar_cluster_labels'
            text.id = 1000 + int(cluster['id'])
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = float(cx)
            text.pose.position.y = float(cy)
            text.pose.position.z = 0.75
            text.pose.orientation.w = 1.0
            text.scale.z = 0.20
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.color.a = 1.0
            text.text = f"C{cluster['id']} {cluster['distance']:.1f}m"
            text.lifetime.nanosec = 300000000
            marker_array.markers.append(text)

        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = LidarClusterNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()