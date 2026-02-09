// File: src/run_once.cpp

#include "go2_control_cpp/run_once.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace go2_control_cpp
{

RunOnce::RunOnce(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::ControlNode(name, config)
, executed_(false)
{}

BT::PortsList RunOnce::providedPorts()
{
  return {};
}

BT::NodeStatus RunOnce::tick()
{
  // If already executed, succeed without ticking child
  if (executed_)
  {
    return status_;
  }

  // If there is no child, consider it executed
  if (children_nodes_.empty())
  {
    executed_ = true;
    return BT::NodeStatus::SUCCESS;
  }

  // First tick: execute child
  status_ = children_nodes_.front()->executeTick();
  // On completion (either SUCCESS or FAILURE), mark executed and return status
  if (status_ == BT::NodeStatus::SUCCESS || status_ == BT::NodeStatus::FAILURE)
  {
    executed_ = true;
  }
  return status_;
}

}  // namespace go2_control_cpp

// Register this control node so it can be used in XML
BT_REGISTER_NODES(factory)
{
  factory.registerBuilder<go2_control_cpp::RunOnce>(
    "RunOnce",
    [](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<go2_control_cpp::RunOnce>(name, config);
    }
  );
}