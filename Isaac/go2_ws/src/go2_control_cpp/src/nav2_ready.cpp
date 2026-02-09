// File: src/nav2_ready.cpp

#include "go2_control_cpp/nav2_ready.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace go2_control_cpp
{

Nav2Ready::Nav2Ready(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::ControlNode(name, config)
{
  // Read 'topic' port
  getInput("topic", topic_);
  // Retrieve ROS2 node from blackboard
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  // Subscribe to ready signal (transient-local to latch)
  ready_sub_ = node_->create_subscription<std_msgs::msg::Empty>(
    topic_,
    rclcpp::QoS(1).transient_local(),
    [this](std_msgs::msg::Empty::SharedPtr) {
      ready_ = true;
      RCLCPP_INFO(
        node_->get_logger(),
        "Nav2Ready: received ready on '%s'", topic_.c_str());
    }
  );
}

BT::PortsList Nav2Ready::providedPorts()
{
  return { BT::InputPort<std::string>("topic", "nav2 ready topic name") };
}

BT::NodeStatus Nav2Ready::tick()
{
  // Always succeed until ready
  if (!ready_) {
    return BT::NodeStatus::SUCCESS;
  }
  // If no child, succeed
  if (children_nodes_.empty()) {
    return BT::NodeStatus::SUCCESS;
  }
  // Tick the first child
  return children_nodes_.front()->executeTick();
}

}  // namespace go2_control_cpp

// Register this control node
BT_REGISTER_NODES(factory)
{
  factory.registerBuilder<go2_control_cpp::Nav2Ready>(
    "Nav2Ready",
    [](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<go2_control_cpp::Nav2Ready>(name, config);
    }
  );
}
