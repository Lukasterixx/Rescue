#pragma once

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/action_node.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <mutex>

namespace go2_control_cpp
{

struct Doorway {
    cv::Point2f p1, p2;
};

struct Room {
    std::vector<cv::Point2f> corners; 
    std::vector<Doorway> doors;
    std_msgs::msg::ColorRGBA color;
    cv::Point2f center;
    double angle; 
    double width, height; 
};

class TopologicalMapperAction : public BT::StatefulActionNode
{
public:
    TopologicalMapperAction(const std::string& name, const BT::NodeConfiguration& config);
    ~TopologicalMapperAction();

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::thread exec_;

    // ROS Interfaces
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_map_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
    
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Data State
    std::mutex map_mutex_;
    nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;
    std::vector<Room> discovered_rooms_;
    std::vector<Doorway> all_doorways_;
    
    rclcpp::Time last_attempt_time_;
    int last_room_idx_; // Tracks the room we were just inside

    // Params
    std::string global_frame_;
    std::string robot_frame_;
    double resolution_;

    // Core Processing Methods
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    int getRobotRoomIndex(double rx, double ry); // Changed to return index
    bool tryRegenerateRoom(int room_idx, double door_len, double rx, double ry); // NEW
    void discoverNewRoom(double rx, double ry, int init_l = -1, int init_r = -1, int init_t = -1, int init_b = -1);    
    cv::Mat gridToMat(const nav_msgs::msg::OccupancyGrid& grid);
    std_msgs::msg::ColorRGBA getRandomColor();
    void publishVisualizations();
    std::vector<std::pair<int, int>> extractDoorwaysBrush(const cv::Mat& region, int axis, int min_width);
};

} // namespace go2_control_cpp