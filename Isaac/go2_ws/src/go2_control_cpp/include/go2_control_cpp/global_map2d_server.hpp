#ifndef GO2_CONTROL_CPP__GLOBAL_MAP2D_SERVER_HPP_
#define GO2_CONTROL_CPP__GLOBAL_MAP2D_SERVER_HPP_

#include <string>
#include <memory>
#include <optional>
#include <deque>
#include <mutex>
#include <vector>
#include <algorithm>
#include <cmath>

// Behavior Tree
#include "behaviortree_cpp_v3/action_node.h"

// ROS Core
#include "rclcpp/rclcpp.hpp"

// Messages
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

// TF2
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// OpenCV (Minimal includes)
#include <opencv2/core.hpp>

// Nav2 Utils
#include <nav2_util/geometry_utils.hpp>
#include <nav2_map_server/map_io.hpp>
#include "go2_control_cpp/common_types.hpp"

namespace go2_control_cpp
{

class LocalizeSubmap : public BT::SyncActionNode
{
public:
  LocalizeSubmap(const std::string & name, const BT::NodeConfiguration & config);
  
  static BT::PortsList providedPorts();
  
  BT::NodeStatus tick() override;

private:
  // --- ROS Handles ---
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr publish_tf_timer_;

  // --- Parameters & State ---
  std::string maps_dir_param_;
  double waypoint_skip_distance_;
  bool exploration_mode = false;
  
  // Stored Data
  nav_msgs::msg::OccupancyGrid static_map_;
  std::optional<nav_msgs::msg::OccupancyGrid> latest_submap_;
  std::vector<geometry_msgs::msg::PoseStamped> waypoints_;
  geometry_msgs::msg::PoseStamped current_pose_;

  // --- Publishers & Subscribers ---
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr submap_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr global_map_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr waypoints_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr global_corners_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr submap_corners_pub_;

  // --- Transform State ---
  geometry_msgs::msg::TransformStamped last_map2d_tf_;
  std::mutex tf_mutex_;
  
  // Alignment State
  double current_yaw_rad_ = 0.0;
  double current_step_deg_ = 1.0; // Adaptive step size

  // --- Methods ---
  bool updateCurrentPoseFromTF();
  nav_msgs::msg::OccupancyGrid loadMapFromYaml(const std::string & yaml_file);
  void submapCallback(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg);

  // Core Logic
  bool alignSubmapToMap(
    const nav_msgs::msg::OccupancyGrid & submap,
    geometry_msgs::msg::TransformStamped & transform);

  int calculateOverlapScore(
      const std::vector<cv::Point2f>& submap_points,
      double yaw_offset);

  std::vector<geometry_msgs::msg::PoseStamped> convertWaypointsToMapFrame();
  bool shouldSkipTfBroadcast(double skip_distance);
};

}  // namespace go2_control_cpp

#endif  // GO2_CONTROL_CPP__GLOBAL_MAP2D_SERVER_HPP_