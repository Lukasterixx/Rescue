#include "go2_control_cpp/lateral_collision_node.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include <sstream> // Added for string building

namespace go2_control_cpp
{

LateralCollisionNode::LateralCollisionNode(
  const std::string& name, 
  const BT::NodeConfiguration& config)
: BT::ControlNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");

  // Subscribe to the hidden action feedback topic.
  feedback_sub_ = node_->create_subscription<ActionT::Impl::FeedbackMessage>(
    "/navigate_through_poses/_action/feedback", 
    rclcpp::SensorDataQoS(),
    [this](const ActionT::Impl::FeedbackMessage::SharedPtr msg) {
      current_poses_remaining_ = msg->feedback.number_of_poses_remaining;
    });
}

BT::PortsList LateralCollisionNode::providedPorts()
{
  return {
    BT::InputPort<std::vector<int>>("long_edge_indices")
  };
}

void LateralCollisionNode::halt()
{
  BT::ControlNode::haltChildren();
  BT::ControlNode::halt();
}

BT::NodeStatus LateralCollisionNode::tick()
{
  // 1. Get the allowed list
  std::vector<int> allowed_indices;
  if (!getInput("long_edge_indices", allowed_indices)) {
      haltChildren();
      return BT::NodeStatus::SUCCESS;
  }

  // 2. Snapshot current count
  int current_len = current_poses_remaining_.load();

  // 3. Check for match
  bool on_long_edge = false;
  if (current_len != -1 && !allowed_indices.empty()) {
      for (int idx : allowed_indices) {
          if (idx == current_len) {
              on_long_edge = true;
              break;
          }
      }
  }

  // 4. Logic: If NO match, stop children
  if (!on_long_edge)
  {
    if (status() == BT::NodeStatus::RUNNING) {
        RCLCPP_INFO(node_->get_logger(), "[LateralCollisionNode] Logic: Exiting Long Edge. Halting children.");
        haltChildren();
    }
    return BT::NodeStatus::SUCCESS;
  }

  // 5. Run children
  for (size_t i = 0; i < children_nodes_.size(); i++)
  {
    BT::TreeNode * child = children_nodes_[i];
    BT::NodeStatus status = child->executeTick();

    switch (status) {
      case BT::NodeStatus::RUNNING:
        return BT::NodeStatus::SUCCESS;

      case BT::NodeStatus::FAILURE:
        RCLCPP_WARN(node_->get_logger(), "[LateralCollisionNode] Child failed. Halting.");
        haltChildren();
        return BT::NodeStatus::FAILURE;

      case BT::NodeStatus::SUCCESS:
        continue; 

      case BT::NodeStatus::IDLE:
        throw std::logic_error("Child status cannot be IDLE inside tick");
    }
  }

  haltChildren();
  return BT::NodeStatus::SUCCESS;
}

} // namespace go2_control_cpp