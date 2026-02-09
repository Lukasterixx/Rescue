#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <unitree_go/msg/low_state.hpp>

namespace go2_control_cpp
{

class BatteryCheckAction : public BT::StatefulActionNode
{
public:
    BatteryCheckAction(const std::string& name, const BT::NodeConfiguration& config);
    ~BatteryCheckAction() override;

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr sub_;
    std::thread exec_;
    double soc_;
    double threshold_;
};

} // namespace go2_control_cpp
