// File: include/go2_control_cpp/initial_pose.hpp

#pragma once

#include <string>
#include <memory>

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace go2_control_cpp
{

class InitialPose : public BT::SyncActionNode
{
public:
  InitialPose(
    const std::string & name,
    const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("input_topic", "/point_cloud2", "PointCloud2 topic"),
      BT::InputPort<double>("z_height", 0.4, "Base_link height above floor"),
      BT::InputPort<double>("seg_distance_threshold", 0.1, "RANSAC distance threshold"),
      BT::InputPort<double>("seg_axis_tolerance_deg", 10.0, "RANSAC axis angle tolerance"),
      BT::InputPort<int>("seg_max_iterations", 500, "RANSAC max iterations")
    };
  }

  BT::NodeStatus tick() override;

private:
  // subscription callback
  void cloudCb(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  // ROS interfaces
  rclcpp::Node::SharedPtr                                node_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr  pub_;

  // state
  sensor_msgs::msg::PointCloud2::SharedPtr first_cloud_;
  bool done_;

  // ports
  std::string input_topic_;
  double      z_height_;
  double      seg_distance_threshold_;
  double      seg_axis_tolerance_deg_;
  int         seg_max_iterations_;
};

}  // namespace go2_control_cpp
