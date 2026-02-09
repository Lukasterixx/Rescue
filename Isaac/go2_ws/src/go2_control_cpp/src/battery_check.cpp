#include "go2_control_cpp/battery_check.hpp"
#include <behaviortree_cpp_v3/bt_factory.h>

#include <thread>
#include <stdexcept>

namespace go2_control_cpp
{

BatteryCheckAction::BatteryCheckAction(const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config),
      node_(rclcpp::Node::make_shared("battery_check_node")),
      soc_(100) // Default full SOC until first message
{
    sub_ = node_->create_subscription<unitree_go::msg::LowState>(
        "/lowstate", 10,
        [this](const unitree_go::msg::LowState::SharedPtr msg)
        {
            soc_ = msg->bms_state.soc;
        });

    // Start spinning node in background thread
    exec_ = std::thread([this]() { rclcpp::spin(node_); });
}

BatteryCheckAction::~BatteryCheckAction()
{
    rclcpp::shutdown();
    if (exec_.joinable())
    {
        exec_.join();
    }
}

BT::PortsList BatteryCheckAction::providedPorts()
{
    return { BT::InputPort<double>("threshold") };
}

BT::NodeStatus BatteryCheckAction::onStart()
{
    double threshold;
    if (!getInput("threshold", threshold))
    {
        throw BT::RuntimeError("Missing required input [threshold]");
    }
    threshold_ = threshold;
    RCLCPP_INFO(node_->get_logger(), "Battery check started with threshold %.1f%%", threshold_);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus BatteryCheckAction::onRunning()
{
    RCLCPP_INFO(node_->get_logger(), "Current SOC: %.1f%%", soc_);
    if (soc_ < threshold_)
    {
        RCLCPP_WARN(node_->get_logger(),
                    "Battery below threshold: %.1f%% < %.1f%%", soc_, threshold_);
        return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
}

void BatteryCheckAction::onHalted()
{
    RCLCPP_INFO(node_->get_logger(), "Battery check halted");
}

} // namespace go2_control_cpp


// Register node as plugin
BT_REGISTER_NODES(factory)
{ 
  factory.registerBuilder<go2_control_cpp::BatteryCheckAction>(
    "BatteryCheckAction",
    [](auto & name, auto & config) {
      return std::make_unique<go2_control_cpp::BatteryCheckAction>(name, config);
    });
}

