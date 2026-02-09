// new_goal.cpp
#include <behaviortree_cpp_v3/decorator_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/blackboard.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

namespace go2_control_cpp {

class NewGoal : public BT::DecoratorNode {
public:
  NewGoal(const std::string & name, const BT::NodeConfiguration & config)
  : BT::DecoratorNode(name, config) {}

  static BT::PortsList providedPorts() {
    return { BT::InputPort<geometry_msgs::msg::PoseStamped>("goal") };
  }

  BT::NodeStatus tick() override {
    geometry_msgs::msg::PoseStamped current_goal;
    // Log every tick for visibility
    RCLCPP_INFO(rclcpp::get_logger("NewGoal"), "[NewGoal] ticked.");

    // Check for goal input
    if (!getInput<geometry_msgs::msg::PoseStamped>("goal", current_goal)) {
      RCLCPP_INFO(rclcpp::get_logger("NewGoal"),
                  "[NewGoal] No goal on blackboard; skipping child tick.");
      return BT::NodeStatus::FAILURE;
    }

    auto bb = config().blackboard;
    geometry_msgs::msg::PoseStamped last_goal;
    bool has_last = bb->get<geometry_msgs::msg::PoseStamped>("last_goal", last_goal);

    // If this is the first (dummy) goal, store and ignore
    if (!has_last) {
      bb->set("last_goal", current_goal);
      RCLCPP_INFO(rclcpp::get_logger("NewGoal"),
                  "[NewGoal] Ignoring initial dummy goal.");
      return BT::NodeStatus::FAILURE;
    }

    // If goal changed, tick child
    if (isDifferent(current_goal, last_goal)) {
      bb->set("last_goal", current_goal);
      return child_node_->executeTick();
    }

    // Otherwise skip
    return BT::NodeStatus::FAILURE;
  }

private:
  bool isDifferent(
    const geometry_msgs::msg::PoseStamped & a,
    const geometry_msgs::msg::PoseStamped & b) const
  {
    return (a.header.frame_id    != b.header.frame_id)   ||
           (a.pose.position.x    != b.pose.position.x)   ||
           (a.pose.position.y    != b.pose.position.y)   ||
           (a.pose.position.z    != b.pose.position.z)   ||
           (a.pose.orientation.x != b.pose.orientation.x)||
           (a.pose.orientation.y != b.pose.orientation.y)||
           (a.pose.orientation.z != b.pose.orientation.z)||
           (a.pose.orientation.w != b.pose.orientation.w);
  }
};

} // namespace go2_control_cpp

// Register custom BT node
BT_REGISTER_NODES(factory)
{
  factory.registerBuilder<go2_control_cpp::NewGoal>(
    "NewGoal",
    [](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<go2_control_cpp::NewGoal>(name, config);
    });
}
