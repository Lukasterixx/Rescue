#ifndef GO2_CONTROL_CPP__PERMISSION_NODE_HPP_
#define GO2_CONTROL_CPP__PERMISSION_NODE_HPP_

#include <behaviortree_cpp_v3/bt_factory.h>
#include <memory>                 // for std::shared_ptr
#include <string>
#include "rclcpp/rclcpp.hpp"  

namespace go2_control_cpp
{
class PermissionNode : public BT::ConditionNode
{
public:
  PermissionNode(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & config);
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts() 
  { 
    return {
      BT::InputPort<std::shared_ptr<rclcpp::Node>>("node")
    }; 
  }
private:
  std::shared_ptr<rclcpp::Node> node_;
};
}  // namespace go2_control_cpp

#endif  // GO2_CONTROL_CPP__PERMISSION_NODE_HPP_
