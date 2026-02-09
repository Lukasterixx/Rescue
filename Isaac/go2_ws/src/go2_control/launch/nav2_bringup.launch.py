from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('go2_control')
    params   = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    map_yaml = os.path.join(pkg_share, 'maps', 'map2d.yaml')

    # Only nav2_* nodes need cmd_vel remap
    cmd_vel_remap = [
        ('/cmd_vel', '/robot0/cmd_vel'),
    ]

    return LaunchDescription([

        
        Node(
            package='nav2_map_server', executable='map_server',
            name='map_server', output='screen',
            parameters=[{
                'yaml_filename': map_yaml,
                'use_sim_time': True,
                'topic_name': '/map_nav2'
            }],
        ),

        # 2) Nav2 core nodes (planner, controller, behavior, bt, lifecycle)
        Node(
            package='nav2_planner', executable='planner_server',
            name='planner_server', output='screen',
            parameters=[params],
            remappings=cmd_vel_remap
        ),
        Node(
            package='nav2_controller', executable='controller_server',
            name='controller_server', output='screen',
            parameters=[params],
            remappings=cmd_vel_remap
        ),
        Node(
            package='nav2_behaviors', executable='behavior_server',
            name='behavior_server', output='screen',
            parameters=[params],
            remappings=cmd_vel_remap
        ),
        Node(
            package='nav2_bt_navigator', executable='bt_navigator',
            name='bt_navigator', output='screen',
            parameters=[params],
            remappings=cmd_vel_remap
        ),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager', output='screen',
            parameters=[params],
            remappings=cmd_vel_remap
        ),

    ])
