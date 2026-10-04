from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    enable_rviz = LaunchConfiguration('enable_rviz')
    enable_nav2 = LaunchConfiguration('enable_nav2')

    perception_pkg = get_package_share_directory('multi_modal_perception')
    robot_pkg = get_package_share_directory('multi_modal_robot_description')
    bringup_pkg = get_package_share_directory('multi_modal_bringup')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                robot_pkg,
                'launch',
                'gazebo.launch.py'
            )
        )
    )

    navigation_launch = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        bringup_pkg,
                        'launch',
                        'nav2_navigation.launch.py'
                    )
                ),
                condition=IfCondition(enable_nav2)
            )
        ]
    )

    lidar_cluster_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='multi_modal_perception',
                executable='lidar_cluster_node',
                name='lidar_cluster_node',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'scan_topic': '/scan',
                    'cluster_topic': '/lidar/clusters',
                    'marker_topic': '/lidar/cluster_markers',
                    'min_range': 0.15,
                    'max_range': 8.0,
                    'cluster_distance': 0.28,
                    'min_cluster_points': 4,
                    'max_cluster_points': 120,
                    'max_cluster_width': 1.40,
                    'max_cluster_depth': 1.40,
                    'max_cluster_area': 1.60,
                    'max_cluster_aspect_ratio': 4.00,
                }]
            )
        ]
    )

    yolo_detector_node = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='multi_modal_perception',
                executable='yolo_detector_node',
                name='yolo_detector_node',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'image_topic': '/camera/image_raw',
                    'model_name': '/home/ros2/datasets/warehouse_objects_v2/runs/detect/warehouse_yolo_v2/weights/best.pt',
                    'confidence_threshold': 0.35,
                    'publish_annotated_image': True,
                }]
            )
        ]
    )

    camera_lidar_association_node = TimerAction(
        period=6.5,
        actions=[
            Node(
                package='multi_modal_perception',
                executable='camera_lidar_association_node',
                name='camera_lidar_association_node',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'clusters_topic': '/lidar/clusters',
                    'yolo_detections_topic': '/yolo/detections',
                    'camera_info_topic': '/camera/camera_info',
                    'associated_objects_topic': '/fusion/associated_objects',
                    'marker_topic': '/fusion/associated_object_markers',
                    'camera_frame': 'camera',
                    'lidar_frame': 'laser',
                    'projection_mode': 'x_forward',
                    'association_max_age_sec': 0.75,
                    'cluster_height_for_projection': 0.30,
                    'bbox_margin_px': 60.0,
                    'one_yolo_match_per_cluster': True,
                    'default_lidar_only_label': 'static_structure',
                    'min_yolo_confidence_for_association': 0.20,
                }]
            )
        ]
    )

    obstacle_state_classifier_node = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='multi_modal_perception',
                executable='obstacle_state_classifier_node',
                name='obstacle_state_classifier_node',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'input_topic': '/fusion/associated_objects',
                    'classified_topic': '/perception/classified_objects',
                    'marker_topic': '/perception/classified_object_markers',
                    'visual_max_distance': 8.0,
                    'visual_show_static': True,
                    'visual_max_labels': 6,
                }]
            )
        ]
    )

    rviz_node = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='perception_rviz',
                output='screen',
                arguments=[
                    '-d',
                    os.path.join(
                        perception_pkg,
                        'rviz',
                        'perception_debug.rviz'
                    )
                ],
                parameters=[{
                    'use_sim_time': True
                }],
                condition=IfCondition(enable_rviz)
            )
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_rviz',
            default_value='true',
            description='Open RViz with perception_debug.rviz'
        ),
        DeclareLaunchArgument(
            'enable_nav2',
            default_value='true',
            description='Start Nav2 using multi_modal_bringup/nav2_navigation.launch.py'
        ),

        gazebo_launch,
        lidar_cluster_node,
        yolo_detector_node,
        camera_lidar_association_node,
        obstacle_state_classifier_node,
        navigation_launch,
        rviz_node,
    ])