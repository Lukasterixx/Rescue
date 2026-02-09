#include "go2_control_cpp/permission_node.hpp"


namespace go2_control_cpp
{

PermissionNode::PermissionNode(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & config)
: BT::ConditionNode(xml_tag_name, config)
{
}

BT::NodeStatus PermissionNode::tick()
{
  return BT::NodeStatus::SUCCESS;
}

}  // namespace go2_control_cpp

BT_REGISTER_NODES(factory)
{
  // Permission check
  factory.registerBuilder<go2_control_cpp::PermissionNode>(
    "Permission",
    [](auto & name, auto & config) {
      return std::make_unique<go2_control_cpp::PermissionNode>(name, config);
    });
}