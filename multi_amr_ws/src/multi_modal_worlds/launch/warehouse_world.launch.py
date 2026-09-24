import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    worlds_pkg = get_package_share_directory('multi_modal_worlds')
    robot_pkg = get_package_share_directory('multi_modal_robot_description')

    world_path = os.path.join(
        worlds_pkg,
        'worlds',
        'warehouse.sdf'
    )

    models_path = os.path.join(
        worlds_pkg,
        'models'
    )

    existing_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')

    if existing_resource_path:
        os.environ['GZ_SIM_RESOURCE_PATH'] = (
            models_path + ':' + existing_resource_path
        )
    else:
        os.environ['GZ_SIM_RESOURCE_PATH'] = models_path

    urdf_path = os.path.join(
        robot_pkg,
        'urdf',
        'robot_description.urdf'
    )

    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    clean_old_processes = ExecuteProcess(
        cmd=[
            'bash',
            '-c',
            '''
            pkill -9 -f gz || true;
            pkill -9 -f gazebo || true;
            pkill -9 -f parameter_bridge || true;
            pkill -9 -f warehouse_traffic_controller || true;
            pkill -9 -f robot_state_publisher || true;
            sleep 2
            '''
        ],
        output='screen'
    )

    gz_sim = TimerAction(
        period=3.0,
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
        period=5.0,
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
        period=7.0,
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
        period=9.0,
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
        period=11.0,
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='multi_modal_bridge',
                output='screen',
                arguments=[
                    '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',

                    '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                    '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                    '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',

                    '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                    '/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
                    '/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                    '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',

                    '/world/warehouse_world/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',

                    '/model/forklift_1/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                    '/model/forklift_2/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                    '/model/forklift_3/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',

                    '/model/human_1/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                    '/model/human_2/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',

                    '/model/cart_1/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                    '/model/cart_2/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                    '/model/cart_3/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',

                    '/model/forklift_1/enable@std_msgs/msg/Bool@gz.msgs.Boolean',
                    '/model/forklift_2/enable@std_msgs/msg/Bool@gz.msgs.Boolean',
                    '/model/forklift_3/enable@std_msgs/msg/Bool@gz.msgs.Boolean',

                    '/model/human_1/enable@std_msgs/msg/Bool@gz.msgs.Boolean',
                    '/model/human_2/enable@std_msgs/msg/Bool@gz.msgs.Boolean',

                    '/model/cart_1/enable@std_msgs/msg/Bool@gz.msgs.Boolean',
                    '/model/cart_2/enable@std_msgs/msg/Bool@gz.msgs.Boolean',
                    '/model/cart_3/enable@std_msgs/msg/Bool@gz.msgs.Boolean',
                ],
            )
        ]
    )

    warehouse_traffic_controller = TimerAction(
        period=14.0,
        actions=[
            Node(
                package='multi_modal_worlds',
                executable='warehouse_traffic_controller.py',
                name='warehouse_traffic_controller',
                output='screen'
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
        warehouse_traffic_controller,
    ])