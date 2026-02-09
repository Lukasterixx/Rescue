// File: include/go2_control_cpp/walk_forward_action.hpp

#ifndef GO2_CONTROL_CPP__WALK_FORWARD_ACTION_HPP_
#define GO2_CONTROL_CPP__WALK_FORWARD_ACTION_HPP_

#include <behaviortree_cpp_v3/bt_factory.h>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <nav_msgs/msg/path.hpp>


namespace go2_control_cpp
{

class WalkForwardAction : public BT::SyncActionNode
{
public:
  WalkForwardAction(
    const std::string & name,
    const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("distance",   10.0,      "meters ahead to walk"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("goal", "computed goal pose"),
      BT::OutputPort<nav_msgs::msg::Path>("path",           "planned path via ComputePathToPose")
    };
  }

  BT::NodeStatus tick() override;

private:
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<tf2_ros::Buffer>         tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace go2_control_cpp

#endif  // GO2_CONTROL_CPP__WALK_FORWARD_ACTION_HPP_
