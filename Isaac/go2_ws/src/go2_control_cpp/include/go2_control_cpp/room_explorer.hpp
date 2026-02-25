#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <behaviortree_cpp_v3/action_node.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <memory>
#include <thread>
#include <atomic>

#include "go2_control_cpp/topological_mapper.hpp" 

namespace go2_control_cpp
{

// NEW: Pair the goal pose with the physical doorway center for validation
struct FrontierGoal {
    geometry_msgs::msg::PoseStamped pose;
    cv::Point2f door_center;
    go2_control_cpp::Doorway door; // NEW: Store the full door for visualization
};

class RoomExplorerAction : public BT::StatefulActionNode
{
public:
    RoomExplorerAction(const std::string& name, const BT::NodeConfiguration& config);
    ~RoomExplorerAction();
    
    static BT::PortsList providedPorts();
    
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    BT::NodeStatus processExploration();

    rclcpp::Node::SharedPtr node_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav2 = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    // NEW: Publisher for the debugging markers
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_frontier_markers_;
    
    // NEW: Helper method to publish the stack
    void publishFrontiers(const std::string& frame_id);

    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
    GoalHandleNav2::SharedPtr current_goal_handle_;
    std::string current_action_name_;

    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::thread exec_;

    // Tracking for Stale Door Filter
    std::vector<cv::Point2f> known_doors_;
    std::vector<FrontierGoal> frontier_stack_; // UPDATED to custom struct
    
    // Tracking for Dynamic Preemption
    geometry_msgs::msg::PoseStamped current_goal_;
    size_t rooms_at_goal_dispatch_;

    std::atomic<bool> goal_done_;
    BT::NodeStatus nav_result_;
};

} // namespace go2_control_cpp