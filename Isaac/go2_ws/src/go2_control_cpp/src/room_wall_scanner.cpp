#include "go2_control_cpp/room_wall_scanner.hpp"
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <algorithm>

namespace go2_control_cpp
{

RoomWallScannerAction::RoomWallScannerAction(const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config),
      current_room_id_(-1),
      command_duration_sec_(3.0) // Wait 3 seconds per wall for the camera to settle
{
    node_ = rclcpp::Node::make_shared("room_wall_scanner_node");
    
    // Publisher for the arm commands
    pub_arm_cmd_ = node_->create_publisher<geometry_msgs::msg::Pose>("/arm_commands", 10);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    exec_ = std::thread([this]() { executor_->spin(); });
}

RoomWallScannerAction::~RoomWallScannerAction()
{
    if (executor_) executor_->cancel();
    if (exec_.joinable()) exec_.join();
}

BT::PortsList RoomWallScannerAction::providedPorts()
{
    return {
        BT::InputPort<std::shared_ptr<TopologicalTree>>("topological_tree", "Tree exported from the mapper"),
        BT::InputPort<std::string>("global_frame", "odom", "Global frame"),
        BT::InputPort<std::string>("robot_frame", "base_link", "Robot frame")
    };
}

BT::NodeStatus RoomWallScannerAction::onStart()
{
    std::shared_ptr<TopologicalTree> tree;
    if (!getInput("topological_tree", tree) || !tree || tree->nodes.empty()) {
        RCLCPP_WARN(node_->get_logger(), "No topological tree available.");
        return BT::NodeStatus::FAILURE;
    }

    double rx, ry, ryaw;
    if (!getRobotPose(rx, ry, ryaw)) {
        return BT::NodeStatus::FAILURE;
    }

    // Determine which room the robot is currently in
    cv::Point2f robot_pt(rx, ry);
    const TopologicalNode* current_room = nullptr;
    
    for (const auto& node : tree->nodes) {
        if (!node.corners.empty() && cv::pointPolygonTest(node.corners, robot_pt, false) >= 0) {
            current_room = &node;
            break;
        }
    }

    if (!current_room) {
        RCLCPP_WARN(node_->get_logger(), "Robot is not inside any mapped room boundaries.");
        return BT::NodeStatus::FAILURE;
    }

    // If we entered a new room, recalculate targets
    if (current_room->id != current_room_id_) {
        current_room_id_ = current_room->id;
        calculateWallTargets(*current_room);
        last_command_time_ = node_->get_clock()->now() - rclcpp::Duration::from_seconds(command_duration_sec_ + 1.0); // Force immediate execution
    }

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus RoomWallScannerAction::onRunning()
{
    if (target_wall_points_.empty()) {
        RCLCPP_INFO(node_->get_logger(), "Finished scanning all walls in room %d.", current_room_id_);
        return BT::NodeStatus::SUCCESS;
    }

    auto now = node_->get_clock()->now();
    if ((now - last_command_time_).seconds() >= command_duration_sec_) {
        // Pop the next target
        cv::Point2f target_pt = target_wall_points_.back();
        target_wall_points_.pop_back();

        double rx, ry, ryaw;
        if (getRobotPose(rx, ry, ryaw)) {
            // Calculate global angle from robot to the wall point
            double global_yaw = std::atan2(target_pt.y - ry, target_pt.x - rx);
            
            // Calculate relative yaw for the arm (assuming arm base is aligned with robot base_link)
            double local_yaw = global_yaw - ryaw;
            
            // Wrap to [-pi, pi]
            local_yaw = std::atan2(std::sin(local_yaw), std::cos(local_yaw));

            // Construct and publish the arm command pose
            geometry_msgs::msg::Pose arm_cmd;
            
            // Requirements: Z=0.2, distance >= 0.15 relative to arm base (0,0,0)
            double r = 0.15;
            arm_cmd.position.x = r * std::cos(local_yaw);
            arm_cmd.position.y = r * std::sin(local_yaw);
            arm_cmd.position.z = 0.2;

            // Pitch=0, Roll=0, horizontal to ground plane
            tf2::Quaternion q;
            q.setRPY(0.0, 0.0, local_yaw);
            arm_cmd.orientation = tf2::toMsg(q);

            pub_arm_cmd_->publish(arm_cmd);
            last_command_time_ = now;
            
            RCLCPP_INFO(node_->get_logger(), "Commanding arm to face wall at local yaw: %.2f", local_yaw);
        }
    }

    return BT::NodeStatus::RUNNING;
}

void RoomWallScannerAction::onHalted()
{
    RCLCPP_INFO(node_->get_logger(), "Room wall scanner halted.");
}

void RoomWallScannerAction::calculateWallTargets(const TopologicalNode& room)
{
    target_wall_points_.clear();

    if (room.corners.size() < 3) return;

    // Loop through each edge (wall) of the room polygon
    for (size_t i = 0; i < room.corners.size(); ++i) {
        cv::Point2f c1 = room.corners[i];
        cv::Point2f c2 = room.corners[(i + 1) % room.corners.size()];
        double wall_len = cv::norm(c2 - c1);
        
        if (wall_len < 0.1) continue;

        cv::Point2f wall_dir = (c2 - c1) / wall_len;

        // 1. Project all doors onto this wall segment (parameterized t from 0 to 1)
        std::vector<std::pair<double, double>> door_intervals;
        
        for (const auto& door : room.doors) {
            // Check if door is approximately on this wall
            double dist_p1 = std::abs((door.p1.x - c1.x) * wall_dir.y - (door.p1.y - c1.y) * wall_dir.x);
            double dist_p2 = std::abs((door.p2.x - c1.x) * wall_dir.y - (door.p2.y - c1.y) * wall_dir.x);
            
            // Threshold for collinearity (e.g. within 0.2 meters)
            if (dist_p1 < 0.2 && dist_p2 < 0.2) {
                double t1 = (door.p1 - c1).dot(wall_dir) / wall_len;
                double t2 = (door.p2 - c1).dot(wall_dir) / wall_len;
                door_intervals.push_back({std::min(t1, t2), std::max(t1, t2)});
            }
        }

        // 2. Sort and merge overlapping door intervals
        std::sort(door_intervals.begin(), door_intervals.end());
        std::vector<std::pair<double, double>> merged_doors;
        for (const auto& interval : door_intervals) {
            if (merged_doors.empty() || merged_doors.back().second < interval.first) {
                merged_doors.push_back(interval);
            } else {
                merged_doors.back().second = std::max(merged_doors.back().second, interval.second);
            }
        }

        // 3. Find the largest solid gap (wall that isn't a doorway)
        double best_t_mid = 0.5;
        double max_gap = 0.0;
        double current_t = 0.0;

        for (const auto& door : merged_doors) {
            if (door.first > current_t) {
                double gap = door.first - current_t;
                if (gap > max_gap) {
                    max_gap = gap;
                    best_t_mid = current_t + gap / 2.0;
                }
            }
            current_t = std::max(current_t, door.second);
        }

        // Check the remaining gap to the end of the wall (t=1.0)
        if (1.0 - current_t > max_gap) {
            max_gap = 1.0 - current_t;
            best_t_mid = current_t + (1.0 - current_t) / 2.0;
        }

        // If the biggest non-door section is at least 20% of the wall (or 0.5 meters), look at it
        if (max_gap * wall_len > 0.5) {
            cv::Point2f target = c1 + wall_dir * (best_t_mid * wall_len);
            target_wall_points_.push_back(target);
        }
    }
}

bool RoomWallScannerAction::getRobotPose(double& rx, double& ry, double& ryaw)
{
    std::string global_frame, robot_frame;
    getInput("global_frame", global_frame);
    getInput("robot_frame", robot_frame);

    try {
        auto t = tf_buffer_->lookupTransform(global_frame, robot_frame, tf2::TimePointZero);
        rx = t.transform.translation.x;
        ry = t.transform.translation.y;
        
        double r, p;
        tf2::getEulerYPR(t.transform.rotation, ryaw, p, r);
        return true;
    } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, "TF Error in Scanner: %s", ex.what());
        return false;
    }
}

} // namespace go2_control_cpp

// Register the node to the factory
BT_REGISTER_NODES(factory)
{ 
  factory.registerBuilder<go2_control_cpp::RoomWallScannerAction>(
    "RoomWallScannerAction",
    [](auto & name, auto & config) {
      return std::make_unique<go2_control_cpp::RoomWallScannerAction>(name, config);
    });
}