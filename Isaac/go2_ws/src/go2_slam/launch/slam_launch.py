from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # launch arguments
    slam_params_file = LaunchConfiguration('slam_params_file')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_sim_time     = LaunchConfiguration('use_sim_time')

    slam_params_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('go2_slam'),
            'config',
            'slam.yaml'
        ]),
        description='Path to SLAM Toolbox parameters file'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('go2_slam'),
            'config',
            'rviz2_cfg.rviz'
        ]),
        description='Path to RViz2 config file'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulated /clock'
    )

    # SLAM Toolbox node
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[ slam_params_file, { 'use_sim_time': use_sim_time } ]
    )

    # RViz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[
            '-d', rviz_config_file
        ],
        parameters=[ { 'use_sim_time': use_sim_time } ]
    )

    return LaunchDescription([
        slam_params_arg,
        rviz_config_arg,
        use_sim_time_arg,
        slam_node,
        rviz_node,
    ])
