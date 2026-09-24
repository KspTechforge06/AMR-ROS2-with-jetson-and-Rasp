import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    worlds_pkg = get_package_share_directory('multi_modal_worlds')
    robot_pkg = get_package_share_directory('multi_modal_robot_description')
    bringup_pkg = get_package_share_directory('multi_modal_bringup')

    world_path = os.path.join(worlds_pkg, 'worlds', 'warehouse_mapping.sdf')
    urdf_path = os.path.join(robot_pkg, 'urdf', 'robot_description.urdf')
    rviz_config = os.path.join(bringup_pkg, 'rviz', 'mapping.rviz')

    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    clean_old_processes = ExecuteProcess(
        cmd=[
            'bash',
            '-c',
            """
            pkill -9 -f gz || true;
            pkill -9 -f gazebo || true;
            pkill -9 -f parameter_bridge || true;
            pkill -9 -f robot_state_publisher || true;
            pkill -9 -f rviz2 || true;
            pkill -9 -f slam_toolbox || true;
            sleep 2
            """
        ],
        output='screen'
    )

    gz_sim = TimerAction(
        period=2.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('ros_gz_sim'),
                        'launch',
                        'gz_sim.launch.py'
                    )
                ),
                launch_arguments={
                    'gz_args': f'-r {world_path}'
                }.items()
            )
        ]
    )

    robot_state_publisher = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{
                    'robot_description': robot_desc,
                    'use_sim_time': True
                }]
            )
        ]
    )

    cleanup_robot = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'gz',
                    'service',
                    '-s',
                    '/world/warehouse_world/remove',
                    '--reqtype',
                    'gz.msgs.Entity',
                    '--reptype',
                    'gz.msgs.Boolean',
                    '--timeout',
                    '1000',
                    '--req',
                    'name: "fwr" type: 2'
                ],
                output='screen'
            )
        ]
    )

    spawn_robot = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-world', 'warehouse_world',
                    '-name', 'fwr',
                    '-topic', 'robot_description',
                    '-x', '-1.2',
                    '-y', '0.8',
                    '-z', '0.15'
                ],
                output='screen'
            )
        ]
    )

    bridge = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='ros_gz_bridge',
                output='screen',
                arguments=[
                    '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                    '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                    '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
                    '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                    '/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
                    '/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                    '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
                    '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
                    '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
                    '/world/warehouse_world/dynamic_pose/info@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
                ],
            )
        ]
    )

    slam_toolbox_node = TimerAction(
        period=11.0,
        actions=[
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'odom_frame': 'odom',
                    'map_frame': 'map',
                    'base_frame': 'base_link',
                    'scan_topic': '/scan',
                    'mode': 'mapping',
                    'map_update_interval': 1.0,
                    'transform_publish_period': 0.05,
                    'minimum_time_interval': 0.2,
                    'resolution': 0.05,
                    'max_laser_range': 12.0,
                    'minimum_travel_distance': 0.05,
                    'minimum_travel_heading': 0.05,
                    'do_loop_closing': True
                }]
            )
        ]
    )

    configure_slam = TimerAction(
        period=14.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'configure'],
                output='screen'
            )
        ]
    )

    activate_slam = TimerAction(
        period=16.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'activate'],
                output='screen'
            )
        ]
    )

    rviz_node = TimerAction(
        period=18.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2_mapping',
                output='screen',
                arguments=['-d', rviz_config],
                parameters=[{'use_sim_time': True}]
            )
        ]
    )

    return LaunchDescription([
        clean_old_processes,
        gz_sim,
        robot_state_publisher,
        cleanup_robot,
        spawn_robot,
        bridge,
        slam_toolbox_node,
        configure_slam,
        activate_slam,
        rviz_node,
    ])