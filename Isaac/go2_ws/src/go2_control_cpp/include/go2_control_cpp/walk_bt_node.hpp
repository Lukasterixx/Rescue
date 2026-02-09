// include/go2_control_cpp/walk_bt_node.hpp
#ifndef GO2_CONTROL_CPP__WALK_BT_NODE_HPP_
#define GO2_CONTROL_CPP__WALK_BT_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <memory>

namespace go2_control_cpp
{

class WalkBTNode : public rclcpp::Node
{
public:
  using SharedPtr = std::shared_ptr<WalkBTNode>;

  /// Constructor registers the ROS node name only
  WalkBTNode();

  /**
   * @brief Initialize the BehaviorTree: register custom nodes, load XML, and start ticking.
   * @param self A shared_ptr to this, to keep the node alive in timers.
   */
  void init(SharedPtr self);

private:
  BT::BehaviorTreeFactory    factory_;
  BT::Tree                   tree_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string maps_dir_;
};

}  // namespace go2_control_cpp

#endif  // GO2_CONTROL_CPP__WALK_BT_NODE_HPP_
