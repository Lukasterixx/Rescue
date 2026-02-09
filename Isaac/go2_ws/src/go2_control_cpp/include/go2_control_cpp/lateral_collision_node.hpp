#pragma once

#include <atomic>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/control_node.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"

namespace go2_control_cpp
{

class LateralCollisionNode : public BT::ControlNode
{
public:
  LateralCollisionNode(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
  void halt() override;

private:
  rclcpp::Node::SharedPtr node_;
  
  // We subscribe to the action feedback message wrapper
  using ActionT = nav2_msgs::action::NavigateThroughPoses;
  rclcpp::Subscription<ActionT::Impl::FeedbackMessage>::SharedPtr feedback_sub_;

  // Atomic ensures thread safety between the ROS callback and the BT tick
  std::atomic<int> current_poses_remaining_{-1};
};

}  // namespace go2_control_cpp