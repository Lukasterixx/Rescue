#pragma once

#include <string>
#include <vector>
#include <cstdint>
#include <memory>
#include <filesystem>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>

namespace go2_control_cpp
{

class FlattenPointCloudToMap2D : public BT::ActionNodeBase
{
public:
  FlattenPointCloudToMap2D(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("resolution",            0.1,   "Grid cell resolution (m)"),
      BT::InputPort<double>("max_width_m",           200.0, "Grid max width (m)"),
      BT::InputPort<double>("max_height_m",          200.0, "Grid max height (m)"),
      BT::InputPort<double>("ground_height_threshold", 0.2,"Points above this z mark as obstacles"),
      BT::InputPort<std::string>("cloud_topic",      "/point_cloud2", "PointCloud2 topic"),
      BT::InputPort<std::string>("pose_topic",       "/current_pose", "Robot pose topic"),
      BT::InputPort<std::string>("map2d_topic",      "/map2d",        "Output map topic"),
      BT::InputPort<int>("queue_size",               10,    "QoS queue size"),
      BT::InputPort<double>("update_radius_m",       20.0,  "Patch update radius (m)"),
      BT::InputPort<std::string>(
        "maps_dir",         
        "/home/lukas/P2Dingo/Isaac/go2_ws/src/go2_control_cpp/maps",        
        "map directory"
      )
    };
  }

  // ActionNodeBase interface
  BT::NodeStatus tick() override;
  void halt() override;

private:
  void loadGrid();
  void publishFullGrid();
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  // ROS node handle
  rclcpp::Node::SharedPtr node_;

  // Configuration
  std::string maps_dir_;
  double      resolution_;
  double      max_width_m_;
  double      max_height_m_;
  double      ground_height_thresh_;
  std::string cloud_topic_;
  std::string pose_topic_;
  std::string map2d_topic_;
  int         queue_size_;
  double      update_radius_m_;

  // Grid data
  size_t             global_w_, global_h_;
  double             origin_x_, origin_y_;
  std::vector<int8_t> grid_;

  // ROS interfaces
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr  pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr    cloud_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr       publisher_;

  // State
  geometry_msgs::msg::Pose                                         current_pose_;
  rclcpp::Time                                                     current_pose_stamp_;
  bool                                                             have_pose_{false};
  bool                                                             have_cloud_{false};
  sensor_msgs::msg::PointCloud2::SharedPtr                         cloud_msg_;
};

}  // namespace go2_control_cpp
