cmake_minimum_required(VERSION 3.5)
project(go2_control_cpp)

### 1) find all the ROS 2 / Nav2 / BT.CPP deps
find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rcpputils REQUIRED)
find_package(nav2_core REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nav2_map_server REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system)
# --- CRITICAL FIX: Force usage of NVIDIA/Jetson OpenCV (4.8) ---
set(OpenCV_DIR "/usr/lib/aarch64-linux-gnu/cmake/opencv4")
# ---------------------------------------------------------------
find_package(OpenCV REQUIRED COMPONENTS core imgproc video imgcodecs)     # for ECC
find_package(visualization_msgs REQUIRED)
find_package(unitree_api REQUIRED)
find_package(unitree_go REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(CURL REQUIRED)


# for geometry_msgs::PoseArray and sensor_msgs::PointCloud2
find_package(sensor_msgs          REQUIRED)

# for tf2::fromMsg() on geometry_msgs::msg::Quaternion
find_package(tf2                  REQUIRED)
find_package(tf2_geometry_msgs    REQUIRED)

# Dependencies for PathPlanner
find_package(yaml-cpp REQUIRED)

include_directories(include)

add_compile_definitions(BOOST_ALLOW_DEPRECATED_HEADERS)


# ───────────────────────────────────────────────────────────────────────────────
# pcl stuff
# ───────────────────────────────────────────────────────────────────────────────
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io registration)
find_package(PCL REQUIRED COMPONENTS common filters)

# make sure PCL include dirs are visible
include_directories(
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# ----------------------------------------------------------------------
# Build initial Pose as a standalone node
# ----------------------------------------------------------------------
add_executable(initial_pose
  src/initial_pose_node.cpp
)

ament_target_dependencies(initial_pose
  rclcpp
  geometry_msgs
  sensor_msgs
  pcl_conversions
  tf2
  Eigen3
)

# Link in PCL libraries
target_link_libraries(initial_pose
  ${PCL_LIBRARIES}
)

install(
  TARGETS initial_pose
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

### 2) aggregate into a variable for reuse
set(common_deps
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  rclcpp_components
  std_msgs
  geometry_msgs
  nav2_behavior_tree
  nav_msgs
  nav2_msgs
  behaviortree_cpp_v3
  std_srvs
  nav2_util
  nav2_core
  tf2_ros
  pluginlib
  rcpputils
)

### 3) bring in Nav2‐package defaults
nav2_package()

# ----------------------------------------------------------------------
# Build flatten as a standalone node
# ----------------------------------------------------------------------

add_executable(flatten_node src/flatten_node.cpp)
ament_target_dependencies(flatten_node 
  ${common_deps}
  pcl_conversions
)
install(
  TARGETS flatten_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

target_include_directories(flatten_node PUBLIC ${PCL_INCLUDE_DIRS})

target_link_libraries(flatten_node ${PCL_LIBRARIES})

# ----------------------------------------------------------------------
# Camera Driver (Thermal + Video Download)
# ----------------------------------------------------------------------
add_executable(camera_driver src/camera_driver.cpp)

ament_target_dependencies(camera_driver
  rclcpp
  geometry_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  ament_index_cpp  # <--- Make sure this is listed here
)

# CRITICAL FIX: Link against Libcurl
target_link_libraries(camera_driver
  ${CURL_LIBRARIES}
)

install(
  TARGETS camera_driver
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# ----------------------------------------------------------------------
# Get Recordings Node
# ----------------------------------------------------------------------
add_executable(get_recordings src/get_recordings.cpp)

# Link against CURL and Ament Index (for share directory path)
target_link_libraries(get_recordings
  ${CURL_LIBRARIES}
  ament_index_cpp::ament_index_cpp
)

# Standard ROS 2 install
install(TARGETS get_recordings
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# ----------------------------------------
# 4) One library per BT‐node
# ----------------------------------------
# PermissionNode
add_library(go2_permission_node SHARED src/permission_node.cpp)
ament_target_dependencies(go2_permission_node ${common_deps})
target_compile_definitions(go2_permission_node PRIVATE BT_PLUGIN_EXPORT)

# WalkForwardAction
add_library(go2_walk_forward_node SHARED src/walk_forward_action.cpp)
ament_target_dependencies(go2_walk_forward_node ${common_deps})
target_compile_definitions(go2_walk_forward_node PRIVATE BT_PLUGIN_EXPORT)

# SLAM Launch
add_library(go2_slam_launch_node SHARED src/slam_launch_node.cpp)
ament_target_dependencies(go2_slam_launch_node ${common_deps})
target_link_libraries(go2_slam_launch_node
  Boost::filesystem
  Boost::system
)
target_compile_definitions(go2_slam_launch_node PRIVATE BT_PLUGIN_EXPORT)

# FlattenPointCloudToMap2D
add_library(go2_flatten_pc2d_node SHARED src/flatten_pointcloud_to_map2d_node.cpp)
ament_target_dependencies(go2_flatten_pc2d_node 
  ${common_deps}
  sensor_msgs
  pcl_conversions  
)
target_link_libraries(go2_flatten_pc2d_node
  ${PCL_LIBRARIES}       # <— and this
)
target_compile_definitions(go2_flatten_pc2d_node PRIVATE BT_PLUGIN_EXPORT)

# Map2DSaver
add_library(go2_map2d_saver_node SHARED src/map2d_saver_node.cpp)
ament_target_dependencies(go2_map2d_saver_node 
  ${common_deps} 
  nav2_map_server
)
target_compile_definitions(go2_map2d_saver_node PRIVATE BT_PLUGIN_EXPORT)

# InitialPose
add_library(go2_initial_pose_node SHARED src/initial_pose.cpp)
ament_target_dependencies(go2_initial_pose_node 
  ${common_deps} 
  sensor_msgs
  pcl_conversions
)
target_compile_definitions(go2_initial_pose_node PRIVATE BT_PLUGIN_EXPORT)

# PathPlanner node: panel detection & waypoint generation
add_library(go2_path_planner_node SHARED src/path_planner.cpp)
ament_target_dependencies(go2_path_planner_node 
  ${common_deps}
)
target_compile_definitions(go2_path_planner_node PRIVATE BT_PLUGIN_EXPORT)

target_link_libraries(go2_path_planner_node
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  yaml-cpp
)

# Nav2ReadyCondition BT node
add_library(go2_nav2_ready_condition SHARED src/nav2_ready.cpp)
ament_target_dependencies(go2_nav2_ready_condition
  ${common_deps}  
)
target_compile_definitions(go2_nav2_ready_condition PRIVATE BT_PLUGIN_EXPORT)

# NewGoal BT node
add_library(go2_new_goal_node SHARED src/new_goal.cpp)
ament_target_dependencies(go2_new_goal_node
  ${common_deps}  
)
target_compile_definitions(go2_new_goal_node PRIVATE BT_PLUGIN_EXPORT)

# RunOnce BT node
add_library(go2_run_once_node SHARED src/run_once.cpp)
ament_target_dependencies(go2_run_once_node
  ${common_deps}  
)
target_compile_definitions(go2_run_once_node PRIVATE BT_PLUGIN_EXPORT)

# LateralCenter BT node
add_library(go2_lateral_collision_node SHARED src/lateral_collision_node.cpp)
ament_target_dependencies(go2_lateral_collision_node
  ${common_deps}  
)
target_compile_definitions(go2_lateral_collision_node PRIVATE BT_PLUGIN_EXPORT)

# CameraController BT node
add_library(go2_camera_controller_node SHARED src/camera_controller.cpp)
ament_target_dependencies(go2_camera_controller_node
  ${common_deps}
  ament_index_cpp  
)
target_compile_definitions(go2_camera_controller_node PRIVATE BT_PLUGIN_EXPORT)

# InitialAlignment BT node
add_library(go2_initial_alignment_node SHARED src/initial_alignment.cpp)
ament_target_dependencies(go2_initial_alignment_node
  ${common_deps}
  pcl_conversions  
)
target_compile_definitions(go2_initial_alignment_node PRIVATE BT_PLUGIN_EXPORT)

# Global Map2d Server BT node
add_library(go2_map2d_server_node SHARED src/global_map2d_server.cpp)
ament_target_dependencies(go2_map2d_server_node
  ${common_deps}
  pcl_conversions
  sensor_msgs
)
# Link system libs separately
target_link_libraries(go2_map2d_server_node
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  yaml-cpp
)

target_link_libraries(go2_map2d_server_node
  ${OpenCV_LIBS}
)

target_compile_definitions(go2_map2d_server_node PRIVATE BT_PLUGIN_EXPORT)

# DataServer BT node
add_library(go2_data_server_node SHARED src/data_server.cpp)
ament_target_dependencies(go2_data_server_node
  ${common_deps}
  sensor_msgs
  pcl_conversions
  tf2
  tf2_geometry_msgs
)
target_compile_definitions(go2_data_server_node PRIVATE BT_PLUGIN_EXPORT)
target_link_libraries(go2_data_server_node
  ${PCL_LIBRARIES}
)

# FrontierExplore BT node
add_library(go2_frontier_explorer_node SHARED src/frontier_explorer.cpp)
target_include_directories(go2_frontier_explorer_node PUBLIC include)
ament_target_dependencies(go2_frontier_explorer_node
  ${common_deps}
  tf2
  tf2_geometry_msgs
  visualization_msgs
)
target_compile_definitions(go2_frontier_explorer_node PRIVATE BT_PLUGIN_EXPORT)
target_link_libraries(go2_frontier_explorer_node
  ${OpenCV_LIBS}
)

# DistanceTickController BT control node
add_library(go2_distance_tick_controller_node SHARED
  src/distance_controller.cpp
)
target_include_directories(go2_distance_tick_controller_node PUBLIC include)
ament_target_dependencies(go2_distance_tick_controller_node
  ${common_deps}
  tf2
  tf2_geometry_msgs
)
target_compile_definitions(go2_distance_tick_controller_node PRIVATE BT_PLUGIN_EXPORT)

# CmdVel -> Unitree Sport bridge (composable component)
add_library(cmdvel_to_unitree SHARED src/cmdvel_to_unitree.cpp)
ament_target_dependencies(cmdvel_to_unitree
  rclcpp
  rclcpp_components
  geometry_msgs
  unitree_api
)

# Register as a component so you can load it into your container
rclcpp_components_register_nodes(cmdvel_to_unitree
  "go2_control_cpp::CmdVelToUnitree"
)

install(
  TARGETS cmdvel_to_unitree
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


# SportClient helper vendored from unitree_ros2 examples
add_library(unitree_sport_client_helper
  src/ros2_sport_client.cpp
)
target_include_directories(unitree_sport_client_helper PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
ament_target_dependencies(unitree_sport_client_helper
  rclcpp
  unitree_api
)

# Link the helper into your bridge component/exe
target_link_libraries(cmdvel_to_unitree
  unitree_sport_client_helper
)

# BatteryCheck BT node
add_library(go2_battery_check_node SHARED src/battery_check.cpp)
ament_target_dependencies(go2_battery_check_node
  ${common_deps}
  unitree_go          # ← or unitree_go / unitree_ros to match the find_package above
)
target_include_directories(go2_battery_check_node PUBLIC include)
target_compile_definitions(go2_battery_check_node PRIVATE BT_PLUGIN_EXPORT)

# ReturnToBase BT node
add_library(go2_return_to_base_node SHARED src/return_to_base.cpp)
ament_target_dependencies(go2_return_to_base_node
  ${common_deps}
  tf2
  tf2_geometry_msgs
)
target_include_directories(go2_return_to_base_node PUBLIC include)
target_compile_definitions(go2_return_to_base_node PRIVATE BT_PLUGIN_EXPORT)


install(
  TARGETS
    go2_permission_node
    go2_walk_forward_node
    go2_slam_launch_node
    go2_flatten_pc2d_node
    go2_map2d_saver_node
    go2_initial_pose_node
    go2_path_planner_node
    go2_nav2_ready_condition
    go2_new_goal_node
    go2_run_once_node
    go2_lateral_collision_node
    go2_initial_alignment_node
    go2_map2d_server_node
    go2_data_server_node
    go2_frontier_explorer_node
    go2_distance_tick_controller_node
    go2_battery_check_node
    go2_return_to_base_node
    go2_camera_controller_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ----------------------------------------
# 5) Navigator plugin: go2_navigator
# ----------------------------------------
add_library(go2_control_navigator SHARED
  src/go2_navigator.cpp
)
install(
  TARGETS go2_control_navigator
  LIBRARY DESTINATION lib
)

ament_target_dependencies(go2_control_navigator
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  pluginlib
  nav2_util
  nav2_core
  nav2_behavior_tree
  geometry_msgs
  nav_msgs
)

# ----------------------------------------
# 5.1) Navigator plugin: go2_navigator_poses
# ----------------------------------------
add_library(go2_control_navigator_poses SHARED
  src/go2_navigator_poses.cpp
)
install(
  TARGETS go2_control_navigator_poses
  LIBRARY DESTINATION lib
)

ament_target_dependencies(go2_control_navigator_poses
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  pluginlib
  nav2_util
  nav2_core
  nav2_behavior_tree
  geometry_msgs
  nav_msgs
)

# ----------------------------------------
# 5.5) Node that launches the navigator
# ----------------------------------------
add_library(bt_go2 SHARED src/bt_go2.cpp)
ament_target_dependencies(bt_go2 ${common_deps})

install(
  TARGETS bt_go2
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# register as an rclcpp component
rclcpp_components_register_nodes(bt_go2
  "go2_control_cpp::BtGo2")


# ----------------------------------------
# 6) Export plugin description for Nav2
# ----------------------------------------
# Make sure navigator_plugins.xml lists your Go2Navigator class
pluginlib_export_plugin_description_file(
  go2_control_cpp
  navigator_plugins.xml
)

# ----------------------------------------
# 7) Install headers, BT trees, configs, launch
# ----------------------------------------
install(
  DIRECTORY include/
            behavior_trees/
            config/
            launch/
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY glim_config/
  DESTINATION share/${PROJECT_NAME}/glim_config
)

install(
  DIRECTORY maps/
  DESTINATION share/${PROJECT_NAME}/maps
)

# ───────────────────────────────────────────────────────────────────────────────
#  X) Finally, build & install the standalone walk_bt_node:
# ───────────────────────────────────────────────────────────────────────────────
add_executable(walk_bt_node src/walk_bt_node.cpp)
ament_target_dependencies(walk_bt_node
  ${common_deps}
  ament_index_cpp
)
target_include_directories(walk_bt_node PRIVATE include)

# link in all of your custom‐BT‐node libraries so their constructors get pulled in
target_link_libraries(walk_bt_node
  go2_permission_node
  go2_walk_forward_node
  go2_slam_launch_node
  go2_flatten_pc2d_node
  go2_map2d_saver_node
  go2_initial_pose_node
  go2_path_planner_node
  go2_nav2_ready_condition
  go2_run_once_node
  go2_lateral_collision_node
  go2_initial_alignment_node
  go2_map2d_server_node
  go2_data_server_node
  go2_frontier_explorer_node
  go2_distance_tick_controller_node
  go2_battery_check_node
  go2_return_to_base_node
  go2_camera_controller_node
)

install(
  TARGETS walk_bt_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# ----------------------------------------
# 8) Export for downstream
# ----------------------------------------
ament_export_include_directories(include)
ament_export_libraries(
  go2_control_navigator
  go2_control_navigator_poses
)
ament_export_dependencies(${common_deps})
ament_package()
