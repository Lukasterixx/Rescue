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


# ----------------------------------------
# 4) One library per BT‐node
# ----------------------------------------

# Map2DSaver
add_library(go2_map2d_saver_node SHARED src/map2d_saver_node.cpp)
ament_target_dependencies(go2_map2d_saver_node 
  ${common_deps} 
  nav2_map_server
)
target_compile_definitions(go2_map2d_saver_node PRIVATE BT_PLUGIN_EXPORT)

# Nav2ReadyCondition BT node
add_library(go2_nav2_ready_condition SHARED src/nav2_ready.cpp)
ament_target_dependencies(go2_nav2_ready_condition
  ${common_deps}  
)
target_compile_definitions(go2_nav2_ready_condition PRIVATE BT_PLUGIN_EXPORT)

# RunOnce BT node
add_library(go2_run_once_node SHARED src/run_once.cpp)
ament_target_dependencies(go2_run_once_node
  ${common_deps}  
)
target_compile_definitions(go2_run_once_node PRIVATE BT_PLUGIN_EXPORT)

# Topological mapper BT node
add_library(go2_topological_mapper_node SHARED src/topological_mapper.cpp)
target_include_directories(go2_topological_mapper_node PUBLIC include)
ament_target_dependencies(go2_topological_mapper_node
  ${common_deps}
  tf2
  tf2_geometry_msgs
  visualization_msgs
)
target_compile_definitions(go2_topological_mapper_node PRIVATE BT_PLUGIN_EXPORT)
target_link_libraries(go2_topological_mapper_node
  ${OpenCV_LIBS}
)

# Room explorer
add_library(go2_room_explorer_node SHARED src/room_explorer.cpp)
target_include_directories(go2_room_explorer_node PUBLIC include)
ament_target_dependencies(go2_room_explorer_node
  ${common_deps}
  tf2
  tf2_geometry_msgs
)
target_compile_definitions(go2_room_explorer_node PRIVATE BT_PLUGIN_EXPORT)
target_link_libraries(go2_room_explorer_node
  ${OpenCV_LIBS}
)

# Room wall scanner
add_library(go2_room_wall_scanner_node SHARED src/room_wall_scanner.cpp) # <-- Fixed source file
target_include_directories(go2_room_wall_scanner_node PUBLIC include)
ament_target_dependencies(go2_room_wall_scanner_node
  ${common_deps}
  tf2
  tf2_geometry_msgs
)
target_compile_definitions(go2_room_wall_scanner_node PRIVATE BT_PLUGIN_EXPORT)
target_link_libraries(go2_room_wall_scanner_node
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

# Link the helper into bridge component/exe
target_link_libraries(cmdvel_to_unitree
  unitree_sport_client_helper
)


install(
  TARGETS
    go2_map2d_saver_node
    go2_nav2_ready_condition
    go2_run_once_node
    go2_topological_mapper_node
    go2_distance_tick_controller_node
    go2_room_explorer_node
    go2_room_wall_scanner_node
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
  DIRECTORY meshes
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
  go2_map2d_saver_node
  go2_nav2_ready_condition
  go2_run_once_node
  go2_topological_mapper_node
  go2_distance_tick_controller_node
  go2_room_explorer_node
  go2_room_wall_scanner_node
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
