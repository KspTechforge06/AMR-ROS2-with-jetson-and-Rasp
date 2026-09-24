import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():

    bringup_dir = get_package_share_directory('multi_modal_bringup')

    nav2_params = os.path.join(bringup_dir, 'config', 'nav2_params.yaml')
    map_file = os.path.join(bringup_dir, 'maps', 'warehouse_map.yaml')


    clean_old_nav2_processes = ExecuteProcess(
        cmd=[
            'bash',
            '-c',
            """
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

    map_server = TimerAction(
        period=1.0,
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
        period=2.0,
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
        period=3.0,
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
        period=5.0,
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
        period=7.0,
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

    return LaunchDescription([
        clean_old_nav2_processes,
        map_server,
        amcl,
        nav2_servers,
        lifecycle_manager,
        publish_initial_pose,
    ])