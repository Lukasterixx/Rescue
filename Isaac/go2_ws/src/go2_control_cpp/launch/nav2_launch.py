import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Get package share directory where the config folder resides
    pkg_share = get_package_share_directory('my_nav2_bringup')  # replace with your package name
    config_dir = os.path.join(pkg_share, 'config')
    default_params_file = os.path.join(config_dir, 'nav2_params.yaml')

    # Launch configuration variables
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    # Declare launch arguments
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack'
    )

    # Map server node subscribing to /map2d topic
    map_server_cmd = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[('map', '/map2d')]
    )

    # Lifecycle manager to bring up all Nav2 nodes
    lifecycle_manager_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time},
            {'autostart': autostart},
            {'node_names': [
                'map_server',
                'amcl',
                'planner_server',
                'controller_server',
                'recoveries_server',
                'bt_navigator'
            ]}
        ]
    )

    return LaunchDescription([
        declare_params_file_cmd,
        declare_use_sim_time_cmd,
        declare_autostart_cmd,
        map_server_cmd,
        lifecycle_manager_cmd
    ])
