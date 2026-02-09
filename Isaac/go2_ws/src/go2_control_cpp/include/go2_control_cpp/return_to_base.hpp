#ifndef GO2_CONTROL_CPP_RETURN_TO_BASE_HPP_
#define GO2_CONTROL_CPP_RETURN_TO_BASE_HPP_

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/action/navigate_through_poses.hpp>
#include <std_msgs/msg/empty.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <thread>
#include <atomic>
#include <string>

namespace go2_control_cpp
{

enum class ReturnState {
    NORMAL_OPERATION,
    ROTATING,
    RETURNING,
    DOCKED
};

class ReturnToBaseAction : public BT::StatefulActionNode
{
public:
    ReturnToBaseAction(const std::string& name, const BT::NodeConfiguration& config);
    ~ReturnToBaseAction() override;

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    void publishOriginGoal();
    bool hasReachedOrigin();
    double getCurrentYaw();
    double normalizeAngle(double angle);
    void sendVelocity(double angular_z);
    
    // Helpers
    void performNav2Reset(); 
    void resetNav2WithForwardGoal();
    void cancelWaypointsNavigation();
    
    // NEW: Function to overwrite goal to stop robot
    void stopNav2WithCurrentPose();
    
    // NEW: Function to send return goal via Action Client
    void sendReturnGoalViaAction();

    void cancelNav2Action();

    // ROS Core
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_return_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_restart_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_waypoints_;
    
    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_goal_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_vel_;

    // Action Clients
    rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SharedPtr client_through_poses_;
    
    // NEW: Client for Return Phase
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr client_to_pose_;

    // NEW: Publisher to clear RViz markers
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_waypoints_;

    // TF2
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
    std::thread exec_;

    // State Variables (Ordered correctly for initialization)
    bool waypoints_active_;
    bool return_goal_active_; // NEW
    std::atomic<ReturnState> current_state_;

    double tolerance_;
    double rot_speed_;
    std::string robot_frame_;
    std::string global_frame_;
    std::string cmd_vel_topic_;

    double target_yaw_;
};

} // namespace go2_control_cpp

#endif // GO2_CONTROL_CPP_RETURN_TO_BASE_HPP_