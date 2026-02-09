// src/slam_launch_node.cpp

#include "go2_control_cpp/slam_launch_node.hpp"
#include <boost/process.hpp>
#include <stdexcept>


namespace go2_control_cpp
{

namespace bp = boost::process;

SLAMLaunchNode::SLAMLaunchNode(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & config)
: StatefulActionNode(xml_tag_name, config)
{}

BT::NodeStatus SLAMLaunchNode::onStart()
{
  // Locate the ros2 CLI
  auto ros2_exe = bp::search_path("ros2");
  if (ros2_exe.empty()) {
    throw BT::RuntimeError("SLAMLaunch: could not find 'ros2' in PATH");
  }

  // Launch the SLAM stack
  slam_proc_ = std::make_unique<bp::child>(
    ros2_exe, "launch", "lidarslam", "lidarslam.launch.py",
    bp::std_out > bp::null,
    bp::std_err > bp::null
  );

  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus SLAMLaunchNode::onRunning()
{
  if (!slam_proc_ || !slam_proc_->running()) {
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

void SLAMLaunchNode::onHalted()
{
  if (slam_proc_ && slam_proc_->running()) {
    slam_proc_->terminate();
    slam_proc_->wait();
  }
  slam_proc_.reset();
}

}  // namespace go2_control_cpp


BT_REGISTER_NODES(factory)
{ 
  // SLAM launch
  factory.registerBuilder<go2_control_cpp::SLAMLaunchNode>(
    "SLAMLaunch",
    [](auto & name, auto & config) {
      return std::make_unique<go2_control_cpp::SLAMLaunchNode>(name, config);
    });
}