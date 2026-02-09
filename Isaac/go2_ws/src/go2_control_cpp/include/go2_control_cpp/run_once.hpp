// File: include/go2_control_cpp/run_once.hpp
#pragma once

#include <string>
#include "behaviortree_cpp_v3/control_node.h"

namespace go2_control_cpp
{

/**
 * @brief Control node that ticks its child exactly once, then always returns the childs first status.
 */
class RunOnce : public BT::ControlNode
{
public:
  /**
   * @brief Construct a new RunOnce control node
   * @param name XML tag name
   * @param config Node configuration
   */
  RunOnce(
    const std::string & name,
    const BT::NodeConfiguration & config);

  /**
   * @brief No additional ports
   */
  static BT::PortsList providedPorts();

  /**
   * @brief On tick: run child once, then always return SUCCESS
   */
  BT::NodeStatus tick() override;

  BT::NodeStatus status_;


private:
  bool executed_;
};

}  // namespace go2_control_cpp