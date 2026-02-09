#pragma once

#include <behaviortree_cpp_v3/bt_factory.h>
#include <boost/process.hpp>
#include <memory>                 // for std::shared_ptr
#include <rclcpp/rclcpp.hpp>


namespace go2_control_cpp
{

class SLAMLaunchNode : public BT::StatefulActionNode
{
public:
  SLAMLaunchNode(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & config);

  // no ports required
  static BT::PortsList providedPorts() 
  { 
    return {
      BT::InputPort<std::shared_ptr<rclcpp::Node>>("node")
    }; 
  }

  // Called once when this node first ticks
  BT::NodeStatus onStart() override;

  // Called on subsequent ticks while status == RUNNING
  BT::NodeStatus onRunning() override;

  // Called if the BT halts this branch (or on shutdown)
  void onHalted() override;

private:
  std::unique_ptr<boost::process::child> slam_proc_;
  std::shared_ptr<rclcpp::Node> node_;
};

}  // namespace go2_control_cpp
