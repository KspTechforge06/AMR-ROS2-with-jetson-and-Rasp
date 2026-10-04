import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    worlds_pkg = get_package_share_directory('multi_modal_worlds')
    robot_pkg = get_package_share_directory('multi_modal_robot_description')
    bringup_dir = get_package_share_directory('multi_modal_bringup')

    world_path = os.path.join(worlds_pkg, 'worlds', 'warehouse.sdf')
    urdf_path = os.path.join(robot_pkg, 'urdf', 'robot_description.urdf')

    nav2_params = os.path.join(bringup_dir, 'config', 'nav2_params.yaml')
    map_file = os.path.join(bringup_dir, 'maps', 'warehouse_map.yaml')
    rviz_config = os.path.join(bringup_dir, 'rviz', 'mapping.rviz')

    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    clean_old_processes = ExecuteProcess(
        cmd=[
            'bash',
            '-c',
            """
            pkill -9 -f gz || true
            pkill -9 -f gazebo || true
            pkill -9 -f parameter_bridge || true
            pkill -9 -f robot_state_publisher || true
            pkill -9 -f rviz2 || true
            pkill -9 -f slam_toolbox || true
            pkill -9 -f warehouse_traffic_controller || true
            pkill -9 -f amcl || true
            pkill -9 -f map_server || true
            pkill -9 -f planner_server || true
            pkill -9 -f controller_server || true
            pkill -9 -f bt_navigator || true
            pkill -9 -f behavior_server || true
            pkill -9 -f smoother_server || true
            pkill -9 -f waypoint_follower || true
            pkill -9 -f velocity_smoother || true
            pkill -9 -f lifecycle_manager || true
            sleep 1
            """
        ],
        output='screen'
    )

    gz_sim = TimerAction(
        period=1.0,
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
        period=2.0,
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

    bridge = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='ros_gz_bridge',
                output='screen',
                arguments=[
                    '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                    '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                    '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
                    '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                    '/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
                    '/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                    '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
                    '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
                    '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
                    '/world/warehouse_world/dynamic_pose/info@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
                    '/model/forklift_1/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                    '/model/forklift_2/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                    '/model/forklift_3/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                    '/model/human_1/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                    '/model/human_2/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                    '/model/cart_1/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                    '/model/cart_2/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                    '/model/cart_3/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                ],
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
                    '-z', '0.30',
                    '-Y', '0.0'
                ],
                output='screen'
            )
        ]
    )

    warehouse_traffic_controller = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='multi_modal_worlds',
                executable='warehouse_traffic_controller.py',
                name='warehouse_traffic_controller',
                output='screen',
                emulate_tty=True,
                parameters=[{'use_sim_time': True}]
            )
        ]
    )
    map_server = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'yaml_filename': map_file
                }]
            )
        ]
    )

    amcl = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                parameters=[nav2_params, {'use_sim_time': True}]
            )
        ]
    )

    nav2_servers = TimerAction(
        period=9.0,
        actions=[
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                parameters=[nav2_params, {'use_sim_time': True}]
            ),
            Node(
                package='nav2_controller',
                executable='controller_server',
                name='controller_server',
                output='screen',
                parameters=[nav2_params, {'use_sim_time': True}]
            ),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                parameters=[nav2_params, {'use_sim_time': True}]
            ),
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                parameters=[nav2_params, {'use_sim_time': True}]
            ),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                parameters=[nav2_params, {'use_sim_time': True}]
            )
        ]
    )

    lifecycle_manager = TimerAction(
        period=11.0,
        actions=[
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'autostart': True,
                    'bond_timeout': 0.0,
                    'node_names': [
                        'map_server',
                        'amcl',
                        'planner_server',
                        'controller_server',
                        'behavior_server',
                        'smoother_server',
                        'bt_navigator'
                    ]
                }]
            )
        ]
    )

    publish_initial_pose = TimerAction(
        period=13.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'topic', 'pub', '--once',
                    '/initialpose',
                    'geometry_msgs/msg/PoseWithCovarianceStamped',
                    """
                    {
                      header: {frame_id: "map"},
                      pose: {
                        pose: {
                          position: {x: -0.012, y: -0.040, z: 0.0},
                          orientation: {z: -0.00243, w: 0.999997}
                        },
                        covariance: [
                          0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                          0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0, 0.0, 0.0, 0.068
                        ]
                      }
                    }
                    """
                ],
                output='screen'
            )
        ]
    )

    rviz_node = TimerAction(
        period=14.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2_navigation',
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
        bridge,
        cleanup_robot,
        spawn_robot,
        warehouse_traffic_controller,
        map_server,
        amcl,
        nav2_servers,
        lifecycle_manager,
        publish_initial_pose,
        rviz_node,
    ])