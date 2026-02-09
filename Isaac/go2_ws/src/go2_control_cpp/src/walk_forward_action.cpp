// File: src/walk_forward_action.cpp

#include "go2_control_cpp/walk_forward_action.hpp"
#include <geometry_msgs/msg/vector3_stamped.hpp>     // <-- ADD THIS
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace go2_control_cpp
{

WalkForwardAction::WalkForwardAction(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
  node_ = config.blackboard
    ->get<std::shared_ptr<rclcpp::Node>>("node");
  tf_buffer_  = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

BT::NodeStatus WalkForwardAction::tick()
{
  // 1) Read how far ahead to walk
  double distance;
  if (!getInput("distance", distance)) {
    throw BT::RuntimeError("WalkForwardAction missing port [distance]");
  }

  // 2) Look up current robot pose in "map" frame
  geometry_msgs::msg::TransformStamped tf_map_base;
  try {
    tf_map_base = tf_buffer_->lookupTransform(
      "map", "base_link", tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(node_->get_logger(),
                 "WalkForwardAction failed to lookup transform: %s",
                 ex.what());
    return BT::NodeStatus::FAILURE;
  }

  // 3) Compute a goal pose 'distance' meters ahead
  geometry_msgs::msg::PoseStamped goal;
  goal.header.stamp    = node_->now();
  goal.header.frame_id = "map";

  geometry_msgs::msg::Vector3Stamped forward_vec;
  forward_vec.header.frame_id = "base_link";
  forward_vec.header.stamp    = node_->now();
  forward_vec.vector.x        = distance;
  forward_vec.vector.y        = 0.0;
  forward_vec.vector.z        = 0.0;

  geometry_msgs::msg::Vector3Stamped world_vec;
  tf2::doTransform(forward_vec, world_vec, tf_map_base);

  goal.pose.position.x    = world_vec.vector.x;
  goal.pose.position.y    = world_vec.vector.y;
  goal.pose.position.z    = tf_map_base.transform.translation.z;
  goal.pose.orientation   = tf_map_base.transform.rotation;

  // 4) Push 'goal' onto the BT blackboard
  setOutput("goal", goal);

  // 5) Let the ComputePathToPose BT node run next in the tree
  //    which will read 'goal' and write 'path'.

  return BT::NodeStatus::SUCCESS;
}

}  // namespace go2_control_cpp


BT_REGISTER_NODES(factory)
{
  // Walk forward
  factory.registerBuilder<go2_control_cpp::WalkForwardAction>(
    "WalkForward",
    [](auto & name, auto & config) {
      return std::make_unique<go2_control_cpp::WalkForwardAction>(name, config);
    });
}