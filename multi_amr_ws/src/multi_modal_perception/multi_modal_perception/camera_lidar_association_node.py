#!/usr/bin/env python3

import json
import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros


class CameraLidarAssociationNode(Node):
    def __init__(self):
        super().__init__('camera_lidar_association_node')

        self.declare_parameter('clusters_topic', '/lidar/clusters')
        self.declare_parameter('yolo_detections_topic', '/yolo/detections')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('associated_objects_topic', '/fusion/associated_objects')
        self.declare_parameter('marker_topic', '/fusion/associated_object_markers')

        self.declare_parameter('camera_frame', 'camera')
        self.declare_parameter('lidar_frame', 'laser')
        self.declare_parameter('projection_mode', 'x_forward')
        self.declare_parameter('association_max_age_sec', 0.35)
        self.declare_parameter('cluster_height_for_projection', 0.0)
        self.declare_parameter('bbox_margin_px', 20.0)
        self.declare_parameter('one_yolo_match_per_cluster', True)
        self.declare_parameter('default_lidar_only_label', 'static_structure')
        self.declare_parameter('min_yolo_confidence_for_association', 0.35)

        self.clusters_topic = self.get_parameter('clusters_topic').value
        self.yolo_detections_topic = self.get_parameter('yolo_detections_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.associated_objects_topic = self.get_parameter('associated_objects_topic').value
        self.marker_topic = self.get_parameter('marker_topic').value

        self.camera_frame = self.get_parameter('camera_frame').value
        self.lidar_frame = self.get_parameter('lidar_frame').value
        self.projection_mode = self.get_parameter('projection_mode').value
        self.association_max_age_sec = float(self.get_parameter('association_max_age_sec').value)
        self.cluster_height_for_projection = float(self.get_parameter('cluster_height_for_projection').value)
        self.bbox_margin_px = float(self.get_parameter('bbox_margin_px').value)
        self.one_yolo_match_per_cluster = bool(self.get_parameter('one_yolo_match_per_cluster').value)
        self.default_lidar_only_label = self.get_parameter('default_lidar_only_label').value
        self.min_yolo_confidence_for_association = float(
            self.get_parameter('min_yolo_confidence_for_association').value
        )

        self.camera_info = None
        self.latest_yolo_packet = None
        self.latest_yolo_time = None
        self.last_log_time = time.time()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.camera_info_sub = self.create_subscription(
            CameraInfo, self.camera_info_topic, self.camera_info_callback, 10
        )
        self.yolo_sub = self.create_subscription(
            String, self.yolo_detections_topic, self.yolo_callback, 10
        )
        self.cluster_sub = self.create_subscription(
            String, self.clusters_topic, self.clusters_callback, 10
        )

        self.associated_pub = self.create_publisher(String, self.associated_objects_topic, 10)
        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, 10)

        self.get_logger().info(
            f'Simple camera-LiDAR association started | clusters={self.clusters_topic} | '
            f'yolo={self.yolo_detections_topic}'
        )

    def camera_info_callback(self, msg):
        self.camera_info = msg

    def yolo_callback(self, msg):
        try:
            packet = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn('Invalid YOLO JSON received.')
            return

        self.latest_yolo_packet = packet
        self.latest_yolo_time = self._stamp_to_float(packet.get('header', {}))

    def clusters_callback(self, msg):
        if self.camera_info is None:
            self.get_logger().warn('Waiting for /camera/camera_info...', throttle_duration_sec=2.0)
            return

        try:
            lidar_packet = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn('Invalid LiDAR cluster JSON received.')
            return

        header = lidar_packet.get('header', {})
        lidar_time = self._stamp_to_float(header)
        clusters = lidar_packet.get('clusters', [])

        yolo_detections = []
        yolo_valid = False

        if self.latest_yolo_packet is not None and self.latest_yolo_time is not None:
            if abs(lidar_time - self.latest_yolo_time) <= self.association_max_age_sec:
                raw_detections = self.latest_yolo_packet.get('detections', [])
                yolo_detections = [
                    det for det in raw_detections
                    if float(det.get('confidence', 0.0)) >= self.min_yolo_confidence_for_association
                ]
                yolo_valid = True

        transform = self.lookup_lidar_to_camera_transform()

        processed_clusters = []
        association_candidates = []

        for cluster_index, cluster in enumerate(clusters):
            centroid = cluster.get('centroid', [0.0, 0.0])
            lidar_point = [
                float(centroid[0]),
                float(centroid[1]),
                self.cluster_height_for_projection,
            ]

            pixel = None
            projection_valid = False

            if transform is not None:
                camera_point = self.transform_point(lidar_point, transform)
                pixel = self.project_camera_point(camera_point)
                projection_valid = pixel is not None

            processed_clusters.append({
                'cluster': cluster,
                'cluster_index': cluster_index,
                'centroid': centroid,
                'pixel': pixel,
                'projection_valid': projection_valid,
            })

            if projection_valid and yolo_detections:
                for det_index, det in enumerate(yolo_detections):
                    bbox = det.get('bbox', None)
                    if bbox is None:
                        continue
                    if not self.pixel_inside_bbox(pixel, bbox, self.bbox_margin_px):
                        continue

                    confidence = float(det.get('confidence', 0.0))
                    center_distance = self.pixel_to_bbox_center_distance(pixel, bbox)
                    score = confidence - 0.001 * center_distance

                    association_candidates.append({
                        'score': score,
                        'confidence': confidence,
                        'center_distance': center_distance,
                        'cluster_index': cluster_index,
                        'det_index': det_index,
                        'det': det,
                    })

        assigned_cluster_to_det = self.assign_detections(association_candidates)
        associated_objects = []

        for item in processed_clusters:
            cluster = item['cluster']
            cluster_index = item['cluster_index']
            centroid = item['centroid']
            pixel = item['pixel']
            projection_valid = item['projection_valid']

            best_det = assigned_cluster_to_det.get(cluster_index)

            if best_det is not None:
                class_label = best_det.get('label', 'unknown_obstacle')
                class_confidence = float(best_det.get('confidence', 0.0))
                fusion_source = 'camera_lidar_associated'
                associated_bbox = best_det.get('bbox', [])
                semantic_object = True
            else:
                class_label = self.default_lidar_only_label
                class_confidence = 0.0
                fusion_source = 'lidar_only'
                associated_bbox = []
                semantic_object = False

            associated_objects.append({
                'object_id': int(cluster.get('id', cluster_index)),
                'class_label': class_label,
                'class_confidence': class_confidence,
                'position_lidar': [float(centroid[0]), float(centroid[1])],
                'velocity_lidar': [0.0, 0.0],
                'speed': 0.0,
                'distance': float(cluster.get('distance', 0.0)),
                'size': [float(v) for v in cluster.get('size', [0.0, 0.0])],
                'num_points': int(cluster.get('num_points', 0)),
                'aspect_ratio': float(cluster.get('aspect_ratio', 0.0)),
                'projected_pixel': pixel if pixel is not None else [],
                'projection_valid': bool(projection_valid),
                'associated_bbox': associated_bbox,
                'fusion_source': fusion_source,
                'semantic_object': bool(semantic_object),
            })

        out = String()
        out.data = json.dumps({
            'header': header,
            'frame_index': int(lidar_packet.get('frame_index', 0)),
            'camera_info_ready': True,
            'tf_ready': transform is not None,
            'yolo_valid': bool(yolo_valid),
            'num_yolo_detections': int(len(yolo_detections)),
            'objects': associated_objects,
        })
        self.associated_pub.publish(out)
        self.publish_markers(header, associated_objects)

        now = time.time()
        if now - self.last_log_time >= 1.0:
            associated_count = sum(
                1 for obj in associated_objects
                if obj['fusion_source'] == 'camera_lidar_associated'
            )
            self.get_logger().info(
                f'Associated clusters: {associated_count}/{len(associated_objects)} | '
                f'YOLO detections: {len(yolo_detections)} | TF: {transform is not None}'
            )
            self.last_log_time = now

    def assign_detections(self, association_candidates):
        assigned_cluster_to_det = {}

        if self.one_yolo_match_per_cluster:
            used_clusters = set()
            used_detections = set()

            association_candidates.sort(
                key=lambda c: (c['score'], c['confidence'], -c['center_distance']),
                reverse=True
            )

            for candidate in association_candidates:
                cluster_index = candidate['cluster_index']
                det_index = candidate['det_index']

                if cluster_index in used_clusters or det_index in used_detections:
                    continue

                assigned_cluster_to_det[cluster_index] = candidate['det']
                used_clusters.add(cluster_index)
                used_detections.add(det_index)
        else:
            best_candidate_by_cluster = {}

            for candidate in association_candidates:
                cluster_index = candidate['cluster_index']
                old_candidate = best_candidate_by_cluster.get(cluster_index)

                if old_candidate is None or candidate['score'] > old_candidate['score']:
                    best_candidate_by_cluster[cluster_index] = candidate

            assigned_cluster_to_det = {
                cluster_index: candidate['det']
                for cluster_index, candidate in best_candidate_by_cluster.items()
            }

        return assigned_cluster_to_det

    def lookup_lidar_to_camera_transform(self):
        try:
            return self.tf_buffer.lookup_transform(
                self.camera_frame,
                self.lidar_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05)
            )
        except Exception as exc:
            self.get_logger().warn(
                f'No TF {self.camera_frame} <- {self.lidar_frame}: {exc}',
                throttle_duration_sec=2.0
            )
            return None

    def transform_point(self, point, transform):
        tx = transform.transform.translation.x
        ty = transform.transform.translation.y
        tz = transform.transform.translation.z

        qx = transform.transform.rotation.x
        qy = transform.transform.rotation.y
        qz = transform.transform.rotation.z
        qw = transform.transform.rotation.w

        x, y, z = point

        r00 = 1.0 - 2.0 * (qy * qy + qz * qz)
        r01 = 2.0 * (qx * qy - qz * qw)
        r02 = 2.0 * (qx * qz + qy * qw)

        r10 = 2.0 * (qx * qy + qz * qw)
        r11 = 1.0 - 2.0 * (qx * qx + qz * qz)
        r12 = 2.0 * (qy * qz - qx * qw)

        r20 = 2.0 * (qx * qz - qy * qw)
        r21 = 2.0 * (qy * qz + qx * qw)
        r22 = 1.0 - 2.0 * (qx * qx + qy * qy)

        cx = r00 * x + r01 * y + r02 * z + tx
        cy = r10 * x + r11 * y + r12 * z + ty
        cz = r20 * x + r21 * y + r22 * z + tz

        return [cx, cy, cz]

    def project_camera_point(self, camera_point):
        fx = float(self.camera_info.k[0])
        fy = float(self.camera_info.k[4])
        cx = float(self.camera_info.k[2])
        cy = float(self.camera_info.k[5])
        width = int(self.camera_info.width)
        height = int(self.camera_info.height)

        x, y, z = camera_point

        if self.projection_mode == 'optical':
            depth = z
            if depth <= 0.05:
                return None
            u = fx * (x / depth) + cx
            v = fy * (y / depth) + cy
        else:
            depth = x
            if depth <= 0.05:
                return None
            u = cx - fx * (y / depth)
            v = cy - fy * (z / depth)

        if u < 0 or u >= width or v < 0 or v >= height:
            return None

        return [float(u), float(v)]

    def pixel_inside_bbox(self, pixel, bbox, margin):
        u, v = pixel
        x1, y1, x2, y2 = bbox

        return (
            (x1 - margin) <= u <= (x2 + margin)
            and
            (y1 - margin) <= v <= (y2 + margin)
        )

    def pixel_to_bbox_center_distance(self, pixel, bbox):
        u, v = pixel
        x1, y1, x2, y2 = bbox

        bbox_cx = 0.5 * (float(x1) + float(x2))
        bbox_cy = 0.5 * (float(y1) + float(y2))

        return math.hypot(float(u) - bbox_cx, float(v) - bbox_cy)

    def publish_markers(self, header, objects):
        marker_array = MarkerArray()
        frame_id = header.get('frame_id', 'laser')

        delete_marker = Marker()
        delete_marker.header.frame_id = frame_id
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        for obj in objects:
            if obj['fusion_source'] != 'camera_lidar_associated':
                continue

            object_id = int(obj['object_id'])
            x, y = obj['position_lidar']
            sx, sy = obj['size']

            cube = Marker()
            cube.header.frame_id = frame_id
            cube.ns = 'camera_lidar_associated_objects'
            cube.id = object_id
            cube.type = Marker.CUBE
            cube.action = Marker.ADD
            cube.pose.position.x = float(x)
            cube.pose.position.y = float(y)
            cube.pose.position.z = 0.35
            cube.pose.orientation.w = 1.0
            cube.scale.x = min(max(float(sx), 0.12), 0.80)
            cube.scale.y = min(max(float(sy), 0.12), 0.80)
            cube.scale.z = 0.45
            cube.color.r = 0.0
            cube.color.g = 1.0
            cube.color.b = 0.0
            cube.color.a = 0.70
            cube.lifetime.nanosec = 300000000
            marker_array.markers.append(cube)

        self.marker_pub.publish(marker_array)

    def _stamp_to_float(self, header):
        sec = float(header.get('stamp_sec', 0.0))
        nanosec = float(header.get('stamp_nanosec', 0.0))
        return sec + nanosec * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = CameraLidarAssociationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()