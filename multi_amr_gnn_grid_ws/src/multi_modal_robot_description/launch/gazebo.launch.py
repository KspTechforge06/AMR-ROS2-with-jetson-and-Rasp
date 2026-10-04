import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    robot_pkg = get_package_share_directory('multi_modal_robot_description')
    worlds_pkg = get_package_share_directory('multi_modal_worlds')

    urdf = os.path.join(
        robot_pkg,
        'urdf',
        'robot_description.urdf'
    )

    world = os.path.join(
        worlds_pkg,
        'worlds',
        'warehouse.sdf'
    )

    with open(urdf, 'r') as f:
        robot_desc = f.read()

    cleanup_processes = ExecuteProcess(
        cmd=[
            'bash',
            '-c',
            '''
            pkill -f gz || true
            pkill -f gazebo || true
            pkill -f robot_state_publisher || true
            pkill -f rviz2 || true
            pkill -f nav2 || true
            pkill -f controller_server || true
            pkill -f planner_server || true
            pkill -f bt_navigator || true
            pkill -f map_server || true
            pkill -f amcl || true
            pkill -f lifecycle_manager || true
            pkill -f warehouse_traffic_controller || true
            pkill -f simple_trajectory || true
            '''
        ],
        output='screen'
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': f'-r {world}'
        }.items()
    )

    delayed_gz_sim = TimerAction(
        period=1.0,
        actions=[gz_sim]
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True
        }]
    )

    cleanup_robot = TimerAction(
        period=2.0,
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
        period=4.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-world',
                    'warehouse_world',
                    '-name',
                    'fwr',
                    '-topic',
                    'robot_description',
                    '-x',
                    '-1.2',
                    '-y',
                    '0.8',
                    '-z',
                    '0.15'
                ],
                output='screen'
            )
        ]
    )

    bridge = TimerAction(
        period=6.0,
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
                    '/model/forklift_1/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                    '/model/forklift_2/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                    '/model/forklift_3/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                    '/model/human_1/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                    '/model/human_2/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                    '/model/cart_1/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                    '/model/cart_2/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                    '/model/cart_3/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                ],
            )
        ]
    )

    traffic_controller = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='multi_modal_worlds',
                executable='warehouse_traffic_controller.py',
                name='warehouse_traffic_controller',
                output='screen'
            )
        ]
    )

    rviz_config = os.path.join(
        robot_pkg,
        'rviz',
        'robot_display.rviz'
    )

    rviz_node = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config],
                parameters=[{'use_sim_time': True}]
            )
        ]
    )

    return LaunchDescription([
        cleanup_processes,
        delayed_gz_sim,
        robot_state_publisher,
        cleanup_robot,
        spawn_robot,
        bridge,
        traffic_controller,
        #rviz_node,
    ])