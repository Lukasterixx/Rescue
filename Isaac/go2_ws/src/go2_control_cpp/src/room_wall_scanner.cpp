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
      command_duration_sec_(2.0) // Wait 3 seconds per wall for the camera to settle
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
    is_sweeping_ = false;
    current_arm_yaw_ = 0.0; // Assume we start forward (or initialize this from joint states if you have them)
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus RoomWallScannerAction::onRunning()
{
    std::shared_ptr<TopologicalTree> tree;
    if (!getInput("topological_tree", tree) || !tree || tree->nodes.empty()) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, 
            "Wall Scanner waiting: No topological tree available yet.");
        return BT::NodeStatus::RUNNING; 
    }

    double rx, ry, ryaw;
    if (!getRobotPose(rx, ry, ryaw)) {
        return BT::NodeStatus::RUNNING; 
    }

    cv::Point2f robot_pt(rx, ry);
    const TopologicalNode* current_room = nullptr;
    for (const auto& node : tree->nodes) {
        if (!node.corners.empty() && cv::pointPolygonTest(node.corners, robot_pt, false) >= 0) {
            current_room = &node;
            break;
        }
    }

    if (!current_room) {
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000, 
            "Wall Scanner waiting: Robot is not inside any mapped room boundaries.");
        return BT::NodeStatus::RUNNING;
    }

    if (current_room->id != current_room_id_) {
        current_room_id_ = current_room->id;
        calculateWallTargets(*current_room);
        last_command_time_ = node_->get_clock()->now() - rclcpp::Duration::from_seconds(command_duration_sec_ + 1.0); 
        is_sweeping_ = false;
    }

    auto now = node_->get_clock()->now();

    // --- TRAJECTORY EXECUTION STATE ---
    if (is_sweeping_) {
        double elapsed = (now - sweep_start_time_).seconds();
        
        // Calculate progress (0.0 to 1.0)
        double t = elapsed / sweep_duration_sec_;
        if (t > 1.0) t = 1.0;

        // Smooth step interpolation (cosine ease-in/ease-out) for softer starts and stops
        double smooth_t = (1.0 - std::cos(t * M_PI)) / 2.0;

        // Calculate the shortest angular distance
        double angle_diff = target_arm_yaw_ - start_arm_yaw_;
        angle_diff = std::atan2(std::sin(angle_diff), std::cos(angle_diff)); // Wrap to [-pi, pi]

        current_arm_yaw_ = start_arm_yaw_ + (smooth_t * angle_diff);

        // Publish the intermediate pose
        geometry_msgs::msg::Pose arm_cmd;
        double r = 0.30;
        arm_cmd.position.x = r * std::cos(current_arm_yaw_);
        arm_cmd.position.y = r * std::sin(current_arm_yaw_);
        arm_cmd.position.z = 0.30;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, current_arm_yaw_); // Enforce strict horizontal orientation constantly
        arm_cmd.orientation = tf2::toMsg(q);

        pub_arm_cmd_->publish(arm_cmd);

        if (t >= 1.0) {
            is_sweeping_ = false; // Sweep finished, start the "stare" timer
            last_command_time_ = now;
            RCLCPP_INFO(node_->get_logger(), "Arm arrived at wall target.");
        }
        
        return BT::NodeStatus::RUNNING;
    }

    // --- IDLE / NEXT TARGET STATE ---
    if (target_wall_points_.empty()) {
        return BT::NodeStatus::RUNNING; 
    }

    if ((now - last_command_time_).seconds() >= command_duration_sec_) {
        cv::Point2f target_pt = target_wall_points_.back();
        target_wall_points_.pop_back();

        double global_yaw = std::atan2(target_pt.y - ry, target_pt.x - rx);
        double local_yaw = global_yaw - ryaw;
        local_yaw = std::atan2(std::sin(local_yaw), std::cos(local_yaw));

        // Setup the sweep trajectory parameters
        start_arm_yaw_ = current_arm_yaw_;
        target_arm_yaw_ = local_yaw;
        sweep_start_time_ = now;
        is_sweeping_ = true;
        
        RCLCPP_INFO(node_->get_logger(), "Initiating sweep from %.2f to %.2f", start_arm_yaw_, target_arm_yaw_);
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