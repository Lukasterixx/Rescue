// File: include/go2_control_cpp/nav2_ready.hpp
#pragma once

#include <string>
#include "behaviortree_cpp_v3/control_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"

namespace go2_control_cpp
{

/**
 * @brief Control node that always returns SUCCESS itself,
 *        and only ticks its first child after a "ready" signal is received.
 */
class Nav2Ready : public BT::ControlNode
{
public:
  /**
   * @brief Construct a new Nav2Ready control node
   * @param name XML tag name
   * @param config Node configuration (contains the ROS node on blackboard)
   */
  Nav2Ready(
    const std::string & name,
    const BT::NodeConfiguration & config);

  /**
   * @brief Ports:
   *        - topic (string): topic name for the "ready" signal
   */
  static BT::PortsList providedPorts();

  /**
   * @brief On tick:
   *        - If not ready_, return SUCCESS (skip child)
   *        - Once ready_, tick the first child and return its status
   */
  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr ready_sub_;
  bool ready_{false};
  std::string topic_;
};

}  // namespace go2_control_cpp
