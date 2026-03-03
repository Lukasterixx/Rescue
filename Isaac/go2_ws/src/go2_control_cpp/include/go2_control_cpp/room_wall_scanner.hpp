#pragma once

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/action_node.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <chrono>

// Include the mapper header to access TopologicalTree and TopologicalNode
#include "go2_control_cpp/topological_mapper.hpp" 

namespace go2_control_cpp
{

class RoomWallScannerAction : public BT::StatefulActionNode
{
public:
    RoomWallScannerAction(const std::string& name, const BT::NodeConfiguration& config);
    ~RoomWallScannerAction();

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::thread exec_;

    // ROS Interfaces
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_arm_cmd_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // State tracking
    int current_room_id_;
    std::vector<cv::Point2f> target_wall_points_;
    rclcpp::Time last_command_time_;
    double command_duration_sec_; // How long to stare at each wall

    // Add these under the private: section
    bool is_sweeping_ = false;
    double current_arm_yaw_ = 0.0;
    double target_arm_yaw_ = 0.0;
    double start_arm_yaw_ = 0.0;
    rclcpp::Time sweep_start_time_;
    double sweep_duration_sec_ = 2.0; // How long it takes to swing the arm

    // Helper methods
    void calculateWallTargets(const TopologicalNode& room);
    bool getRobotPose(double& rx, double& ry, double& ryaw);
};

} // namespace go2_control_cpp