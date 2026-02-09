// src/cmdvel_to_unitree.cpp
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <unitree_api/msg/request.hpp>
#include "common/ros2_sport_client.h"   // your vendored helper

using namespace std::chrono_literals;

namespace go2_control_cpp {

class CmdVelToUnitree : public rclcpp::Node {
public:
  explicit CmdVelToUnitree(const rclcpp::NodeOptions & options)
  : rclcpp::Node("cmdvel_to_unitree", options)
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "robot0/cmd_vel");
    use_stamped_ = declare_parameter<bool>("use_stamped", false);
    
    // Safety: How long to keep moving if Nav2 goes silent?
    cmd_timeout_ = declare_parameter<double>("cmd_timeout", 0.5); 

    req_pub_ = create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);

    // ✅ Construct SportClient with this node
    sport_ = std::make_unique<SportClient>(this);

    // Initial State
    last_cmd_time_ = this->now();
    latest_twist_ = geometry_msgs::msg::Twist(); // Zero init

    // Request Sport mode right away (adjust helper call to your SDK if different)
    {
      unitree_api::msg::Request req;
      sport_->StopMove(req);    
      req_pub_->publish(req);
      RCLCPP_INFO(get_logger(), "Sent Sport mode request on startup");
    }

    // Subscribe to /cmd_vel (Store data only)
    if (use_stamped_) {
      sub_stamped_ = create_subscription<geometry_msgs::msg::TwistStamped>(
        input_topic_, 10,
        [this](geometry_msgs::msg::TwistStamped::SharedPtr msg) { 
            this->latest_twist_ = msg->twist;
            this->last_cmd_time_ = this->now();
        });
    } else {
      sub_twist_ = create_subscription<geometry_msgs::msg::Twist>(
        input_topic_, 10,
        [this](geometry_msgs::msg::Twist::SharedPtr msg) { 
            this->latest_twist_ = *msg; 
            this->last_cmd_time_ = this->now();
        });
    }

    // ✅ TIMER: The Heartbeat
    // Publishes at fixed 20Hz regardless of input rate
    timer_ = create_wall_timer(
      50ms, // 20Hz
      std::bind(&CmdVelToUnitree::controlLoop, this));
  }

private:
  void controlLoop()
  {
    // Safety Watchdog:
    // If we haven't heard from Nav2 in > 0.5s, force a stop.
    // This prevents the robot from running away if the planner crashes.
    double time_since_cmd = (this->now() - last_cmd_time_).seconds();
    
    if (time_since_cmd > cmd_timeout_) {
      // Fade to zero or hard stop? Hard stop is safer here.
      latest_twist_ = geometry_msgs::msg::Twist(); 
    }

    // Construct and Publish the Unitree Request
    unitree_api::msg::Request req;
    sport_->Move(req,
                 static_cast<float>(latest_twist_.linear.x),
                 static_cast<float>(latest_twist_.linear.y),
                 static_cast<float>(latest_twist_.angular.z));
    req_pub_->publish(req);
  }

  // Params
  std::string input_topic_;
  bool use_stamped_{false};
  double cmd_timeout_{0.5};

  // State
  geometry_msgs::msg::Twist latest_twist_;
  rclcpp::Time last_cmd_time_;

  // IO
  rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr req_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_twist_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_stamped_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Client
  std::unique_ptr<SportClient> sport_;
};

} // namespace go2_control_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(go2_control_cpp::CmdVelToUnitree)