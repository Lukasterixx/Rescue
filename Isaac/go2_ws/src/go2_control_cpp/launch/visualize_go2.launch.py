import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # --- 1. Find Package Directories ---
    # Assuming the URDF and RViz config are in 'go2_control_cpp'
    rviz_pkg_path = get_package_share_directory('go2_control_cpp')

    # --- 2. Build File Paths ---
    urdf_file = os.path.join(rviz_pkg_path, 'go2.urdf')
    rviz_config_file = os.path.join(rviz_pkg_path, 'mapping.rviz')

    # --- 3. Read URDF Content ---
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        # Node 1: Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}],
        ),

        # Node 2: RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        ),
    ])
