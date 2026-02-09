#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import has_resource

def glim_available() -> bool:
        # Check if the 'glim_ros' package is in the current ROS environment
        return has_resource('packages', 'glim_ros')

def generate_launch_description():

    ld = LaunchDescription()

    # Paths
    pkg_share   = get_package_share_directory('go2_control_cpp')
    bt_dir      = os.path.join(pkg_share, 'behavior_trees')
    default_params = os.path.join(pkg_share, 'nav2_params.yaml')

    # Launch configurations and arguments
    params_file  = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart    = LaunchConfiguration('autostart')

    declare_params_file = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='Full path to ROS2 parameters file')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock if true')
    declare_autostart = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically lifecycle‐bringup nodes')
    
    param_substitutions = {
        'use_sim_time':               use_sim_time,
        'autostart':                  autostart,
    }
    
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key='',
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

   
    # GLIM SLAM (either internal node or external Docker)
    if glim_available():
        # Case 1: GLIM is built in this workspace (e.g. on the Go2)
        glim_node = Node(
            package='glim_ros',
            executable='glim_rosnode',
            name='glim_rosnode',
            output='screen',
            parameters=[{
                'config_path': os.path.join(pkg_share, 'glim_config'),
            }],
        )
        ld.add_action(glim_node)

    
    # Line‐buffered logging
    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    # Which nodes to bring up via lifecycle manager
    lifecycle_nodes = [
        'planner_server',
        'controller_server',
        'behavior_server',
        #'velocity_smoother',
        'bt_go2', # instead of default bt_navigator
    ]

    # TF remappings
    remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
    ]

    flatten_node = Node(
        package='go2_control_cpp',
        executable='flatten_node',
        name='flatten_node',
        output='screen',
        parameters=[configured_params]  # must include your resolution, topics, etc.
    )

    # --- NEW: Camera Driver Node ---
    camera_driver_node = Node(
        package='go2_control_cpp',
        executable='camera_driver',
        name='camera_driver',
        output='screen',
        # parameters=[configured_params] # Add if you eventually move constants to ROS params
    )

    # 1) Create the component container
    container = Node(
        name='nav2_container',
        package='rclcpp_components',
        executable='component_container_isolated',
        output='screen',
        parameters=[configured_params, {'autostart': autostart}],
        remappings=remappings,
    )

    # 2) Load all the Nav2 composables into it
    load_composables = LoadComposableNodes(
        target_container='nav2_container',
        composable_node_descriptions=[
            # ComposableNode(
            #     package='nav2_map_server',
            #     plugin='nav2_map_server::MapServer',
            #     name='map_server',
            #     parameters=[configured_params],
            #     remappings=remappings,
            # ),
            ComposableNode(
                package='nav2_planner',
                plugin='nav2_planner::PlannerServer',
                name='planner_server',
                parameters=[configured_params],
                remappings=remappings,
            ),
            ComposableNode(
                package='nav2_controller',
                plugin='nav2_controller::ControllerServer',
                name='controller_server',
                parameters=[configured_params],
                remappings=remappings + [('cmd_vel', 'robot0/cmd_vel')],
            ),
            # ComposableNode(
            #     package='nav2_bt_navigator',
            #     plugin='nav2_bt_navigator::BtNavigator',
            #     name='bt_navigator',
            #     parameters=[configured_params],
            #     remappings=remappings,
            # ),
            ComposableNode(
                package='go2_control_cpp',
                plugin='go2_control_cpp::BtGo2',
                name='bt_go2',
                parameters=[
                    configured_params
                ],
            ),
            # (Optional) velocity smoother
            ComposableNode(
                package='nav2_velocity_smoother',
                plugin='nav2_velocity_smoother::VelocitySmoother',
                name='velocity_smoother',
                parameters=[configured_params],
                remappings=remappings + [
                    ('cmd_vel', 'cmd_vel_nav'),
                    ('cmd_vel_smoothed', 'robot0/cmd_vel'),
                ],
            ),
            ComposableNode(
                package='nav2_behaviors',
                plugin='behavior_server::BehaviorServer',
                name='behavior_server',
                parameters=[ configured_params ],
                remappings=remappings,
            ),
            ComposableNode(
                package='go2_control_cpp',
                plugin='go2_control_cpp::CmdVelToUnitree',
                name='cmdvel_to_unitree',
                parameters=[configured_params, {
                    'input_topic': 'robot0/cmd_vel',  # Nav2 output
                    'use_stamped': False              # Set True if TwistStamped
                }],
            ),
            # Lifecycle manager
            ComposableNode(
                package='nav2_lifecycle_manager',
                plugin='nav2_lifecycle_manager::LifecycleManager',
                name='lifecycle_manager_navigation',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'autostart':    autostart,
                    'node_names':   lifecycle_nodes,
                }],
                remappings=remappings,
            ),
        ],
    )

    walk_bt = Node(
        package='go2_control_cpp',
        executable='walk_bt_node',
        name='walk_bt_node',
        output='screen',
        parameters=[
            configured_params,
            {'use_sim_time': use_sim_time}
        ],
        remappings=remappings,
    )

     # Foxglove Bridge node
    # foxglove_bridge_node = Node(
    #     package='foxglove_bridge',
    #     executable='foxglove_bridge',
    #     name='foxglove_bridge',
    #     output='screen',
    #     parameters=[{
    #         "port": 8765,
    #         "address": "0.0.0.0",
    #         # Restrict what’s visible to Foxglove (edit as needed)
    #         # "topic_whitelist": ["/map", "/tf", "/tf_static"],
    #         "client_topic_whitelist": ["/goal_pose"],  # what the browser can publish to
    #         # "tls": True,
    #         # "certfile": "/path/to/fullchain.pem",
    #         # "keyfile": "/path/to/privkey.pem"
    #     }]
    # )

    # ld.add_action(foxglove_bridge_node)

    # 3) Assemble the launch description
    # Force Jetson OpenCV libraries at runtime
    force_opencv_env = SetEnvironmentVariable(
        'LD_PRELOAD', 
        '/lib/libopencv_core.so.408:/lib/libopencv_highgui.so.408:/lib/libopencv_imgproc.so.408:/opt/gtsam_stack/lib/libgtsam.so'
    )
    
    # Add this to your ld
    ld.add_action(force_opencv_env)
    
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(declare_params_file)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_autostart)
    ld.add_action(flatten_node)
    ld.add_action(camera_driver_node)
    ld.add_action(container)
    ld.add_action(load_composables)
    ld.add_action(walk_bt)
    return ld
