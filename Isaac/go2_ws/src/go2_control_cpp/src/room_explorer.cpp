#include "go2_control_cpp/room_explorer.hpp"
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>

namespace go2_control_cpp
{

RoomExplorerAction::RoomExplorerAction(const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config),
      current_action_name_(""),
      rooms_at_goal_dispatch_(0), // Init preemption tracker
      goal_done_(true),
      nav_result_(BT::NodeStatus::IDLE)
{
    node_ = rclcpp::Node::make_shared("room_explorer_node");
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    pub_frontier_markers_ = node_->create_publisher<visualization_msgs::msg::Marker>("/unexplored_doors", 10);

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    exec_ = std::thread([this]() { executor_->spin(); });
}

RoomExplorerAction::~RoomExplorerAction()
{
    if (executor_) executor_->cancel();
    if (exec_.joinable()) exec_.join();
}

BT::PortsList RoomExplorerAction::providedPorts()
{
    return {
        BT::InputPort<std::shared_ptr<TopologicalTree>>("topological_tree", "Tree exported from the mapper"),
        BT::InputPort<std::string>("global_frame", "odom", "Global frame"),
        BT::InputPort<std::string>("robot_frame", "base_link", "Robot frame"),
        BT::OutputPort<geometry_msgs::msg::PoseStamped>("goal_pose", "Target pose through a doorway"),
        BT::InputPort<std::string>("action_name", "go2_navigator", "Name of the Nav2 action server")
    };
}

BT::NodeStatus RoomExplorerAction::onStart()
{
    std::string action_name;
    if (!getInput("action_name", action_name)) action_name = "go2_navigator";

    if (!nav_client_ || current_action_name_ != action_name) {
        RCLCPP_INFO(node_->get_logger(), "Connecting to Nav2 Action Server: '%s'", action_name.c_str());
        nav_client_ = rclcpp_action::create_client<NavigateToPose>(node_, action_name);
        current_action_name_ = action_name;
    }

    goal_done_ = true; 
    return processExploration();
}

BT::NodeStatus RoomExplorerAction::onRunning()
{
    // --- NEW: Dynamic Preemption Logic ---
    if (!goal_done_) {
        std::shared_ptr<TopologicalTree> tree;
        if (getInput("topological_tree", tree) && tree) {
            
            // Did the mapper discover a new room while we were driving?
            if (tree->nodes.size() > rooms_at_goal_dispatch_) {
                std::string global_frame, robot_frame;
                getInput("global_frame", global_frame);
                getInput("robot_frame", robot_frame);

                double rx = 0.0, ry = 0.0;
                try {
                    auto t = tf_buffer_->lookupTransform(global_frame, robot_frame, tf2::TimePointZero);
                    rx = t.transform.translation.x;
                    ry = t.transform.translation.y;

                    // Are we within 0.5m of the target door?
                    double dist = std::hypot(current_goal_.pose.position.x - rx, 
                                             current_goal_.pose.position.y - ry);

                    if (dist < 0.5) {
                        RCLCPP_INFO(node_->get_logger(), "New room added while arriving! Preempting current goal to explore new frontiers immediately.");
                        if (current_goal_handle_) {
                            nav_client_->async_cancel_goal(current_goal_handle_);
                        }
                        
                        // NEW: Update the tracker so we don't spam cancel requests on the next BT tick
                        rooms_at_goal_dispatch_ = tree->nodes.size(); 
                        
                    }
                } catch (const tf2::TransformException & ex) {
                    // Ignore transient TF errors
                }
            }
        }

        // If we didn't preempt and Nav2 is still driving, keep running
        if (!goal_done_) {
            return BT::NodeStatus::RUNNING;
        }
    }
    // ---------------------------------------
    
    return processExploration();
}

BT::NodeStatus RoomExplorerAction::processExploration()
{
    std::shared_ptr<TopologicalTree> tree;
    if (!getInput("topological_tree", tree) || !tree || tree->nodes.empty()) {
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000, 
            "Explorer waiting: Topological tree is empty or not on blackboard yet.");
        return BT::NodeStatus::RUNNING; 
    }

    std::string global_frame, robot_frame;
    getInput("global_frame", global_frame);
    getInput("robot_frame", robot_frame);

    double rx = 0.0, ry = 0.0;
    try {
        auto t = tf_buffer_->lookupTransform(global_frame, robot_frame, tf2::TimePointZero);
        rx = t.transform.translation.x;
        ry = t.transform.translation.y;
    } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, "TF Error: %s", ex.what());
        return BT::NodeStatus::RUNNING; 
    }

    cv::Point2f robot_pt(rx, ry);

    int current_room_id = -1;
    cv::Point2f current_center;
    
    // 1. Primary method: Check if the robot is inside the room's polygon boundary
    for (const auto& node : tree->nodes) {
        if (!node.corners.empty() && cv::pointPolygonTest(node.corners, robot_pt, false) >= 0) {
            current_room_id = node.id;
            current_center = node.center;
            break; // Found our exact room
        }
    }

    // 2. Fallback method: If standing directly in a doorway (outside boundaries), default to nearest center
    if (current_room_id == -1) {
        double min_dist = 1e9;
        for (const auto& node : tree->nodes) {
            double d = cv::norm(node.center - robot_pt);
            if (d < min_dist) {
                min_dist = d;
                current_room_id = node.id;
                current_center = node.center;
            }
        }
    }

    const TopologicalNode* current_node = nullptr;
    for (const auto& node : tree->nodes) {
        if (node.id == current_room_id) {
            current_node = &node;
            break;
        }
    }

    if (!current_node) return BT::NodeStatus::RUNNING;

    // Scan for new doors and push to DFS stack
    for (const auto& door : current_node->doors) {
        cv::Point2f m = (door.p1 + door.p2) / 2.0;
        
        bool known = false;
        for (const auto& kd : known_doors_) {
            if (cv::norm(kd - m) < 0.5) { 
                known = true;
                break;
            }
        }

        if (!known) {
            known_doors_.push_back(m);
            
            cv::Point2f v = door.p2 - door.p1;
            double L = cv::norm(v);
            if (L < 0.01) continue;

            cv::Point2f n1(-v.y / L, v.x / L);
            cv::Point2f n2(v.y / L, -v.x / L);

            double d1 = cv::norm((m + n1) - current_center);
            double d2 = cv::norm((m + n2) - current_center);
            cv::Point2f n_out = (d1 > d2) ? n1 : n2;
            
            cv::Point2f target = m + n_out * (0.75 * L);
            double yaw = std::atan2(n_out.y, n_out.x);

            geometry_msgs::msg::PoseStamped goal;
            goal.header.frame_id = global_frame;
            goal.header.stamp = node_->get_clock()->now();
            goal.pose.position.x = target.x;
            goal.pose.position.y = target.y;
            goal.pose.position.z = 0.0;

            tf2::Quaternion q;
            q.setRPY(0, 0, yaw);
            goal.pose.orientation = tf2::toMsg(q);

            // UPDATED: Push the custom struct to track the physical door
            FrontierGoal fg;
            fg.pose = goal;
            fg.door_center = m;
            fg.door = door;
            frontier_stack_.push_back(fg);
            
            RCLCPP_INFO(node_->get_logger(), "New frontier door discovered! Pushed to DFS stack.");
        }
    }

    // --- NEW: Stale Door Filter via Total Angular Deviation Minimization ---
    FrontierGoal next_frontier;
    bool valid_door_found = false;

    while (!frontier_stack_.empty() && !valid_door_found) {
        next_frontier = frontier_stack_.back();
        frontier_stack_.pop_back();

        // 1. Identify which room this frontier originated from
        const TopologicalNode* parent_node = nullptr;
        double min_room_dist = 1e9;
        for (const auto& n : tree->nodes) {
            double d = cv::norm(n.center - next_frontier.door_center);
            if (d < min_room_dist) {
                min_room_dist = d;
                parent_node = &n;
            }
        }

        if (parent_node) {
            // 2. Gather all frontiers associated with this same room to minimize total deviation globally
            std::vector<FrontierGoal> room_frontiers;
            room_frontiers.push_back(next_frontier);
            
            std::vector<FrontierGoal> other_frontiers;
            for (const auto& fg : frontier_stack_) {
                double fd_min = 1e9;
                int fd_room_id = -1;
                for (const auto& n : tree->nodes) {
                    double dist = cv::norm(n.center - fg.door_center);
                    if (dist < fd_min) { fd_min = dist; fd_room_id = n.id; }
                }
                if (fd_room_id == parent_node->id) {
                    room_frontiers.push_back(fg);
                } else {
                    other_frontiers.push_back(fg);
                }
            }

            // 3. Greedy assignment matching to minimize total angular deviation
            struct MatchPair { int f_idx; int d_idx; double cost; };
            std::vector<MatchPair> pairs;
            
            for (size_t i = 0; i < room_frontiers.size(); ++i) {
                for (size_t j = 0; j < parent_node->doors.size(); ++j) {
                    // Calculate angular difference, properly wrapped to [-pi, pi]
                    double diff = std::abs(std::atan2(
                        std::sin(room_frontiers[i].door.angle - parent_node->doors[j].angle),
                        std::cos(room_frontiers[i].door.angle - parent_node->doors[j].angle)
                    ));
                    pairs.push_back({static_cast<int>(i), static_cast<int>(j), diff});
                }
            }

            // Sort pairs by lowest angular deviation
            std::sort(pairs.begin(), pairs.end(), [](const MatchPair& a, const MatchPair& b) {
                return a.cost < b.cost;
            });

            std::vector<bool> f_assigned(room_frontiers.size(), false);
            std::vector<bool> d_assigned(parent_node->doors.size(), false);
            
            for (const auto& p : pairs) {
                if (!f_assigned[p.f_idx] && !d_assigned[p.d_idx]) {
                    // Maximum allowed deviation: 90 degrees (prevents matching opposite sides of the room)
                    if (p.cost < M_PI / 2.0) {
                        f_assigned[p.f_idx] = true;
                        d_assigned[p.d_idx] = true;
                        
                        room_frontiers[p.f_idx].door = parent_node->doors[p.d_idx];
                        room_frontiers[p.f_idx].door_center = (parent_node->doors[p.d_idx].p1 + parent_node->doors[p.d_idx].p2) / 2.0;
                    }
                }
            }

            // 4. Reconstruct stack and check if the popped frontier survived
            frontier_stack_ = other_frontiers; 
            
            if (f_assigned[0]) {
                valid_door_found = true;
                next_frontier = room_frontiers[0];
            } else {
                RCLCPP_INFO(node_->get_logger(), "Discarded phantom/stale doorway from stack (Map updated).");
            }
            
            // Push remaining assigned room frontiers back onto the stack
            for (size_t i = 1; i < room_frontiers.size(); ++i) {
                if (f_assigned[i]) {
                    frontier_stack_.push_back(room_frontiers[i]);
                }
            }
        } else {
            RCLCPP_INFO(node_->get_logger(), "Discarded frontier: Parent room no longer exists.");
        }
    }
    // ------------------------------

    if (!nav_client_->wait_for_action_server(std::chrono::seconds(1))) {
        RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, "Nav2 Action server not available");
        // Push the valid door back so we don't lose it
        frontier_stack_.push_back(next_frontier);
        return BT::NodeStatus::RUNNING;
    }

    // Save state for preemption
    current_goal_ = next_frontier.pose;
    rooms_at_goal_dispatch_ = tree->nodes.size();
    setOutput("goal_pose", current_goal_);

    NavigateToPose::Goal goal_msg;
    goal_msg.pose = current_goal_;

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    
    send_goal_options.goal_response_callback =
        [this](const GoalHandleNav2::SharedPtr& goal_handle) {
            if (!goal_handle) {
                RCLCPP_ERROR(node_->get_logger(), "Nav2 rejected the goal.");
                goal_done_ = true; 
                nav_result_ = BT::NodeStatus::FAILURE;
            } else {
                current_goal_handle_ = goal_handle;
            }
        };

    send_goal_options.result_callback =
        [this](const GoalHandleNav2::WrappedResult& result) {
            goal_done_ = true; 
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_INFO(node_->get_logger(), "Goal reached naturally. Doorway frontier mapped.");
                nav_result_ = BT::NodeStatus::SUCCESS;
            } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
                // If we pre-empted it ourselves, Nav2 will cancel the goal. This is expected.
                RCLCPP_INFO(node_->get_logger(), "Goal was canceled (likely preempted for new room).");
                nav_result_ = BT::NodeStatus::SUCCESS; 
            } else {
                RCLCPP_WARN(node_->get_logger(), "Nav2 failed/aborted to reach door. Abandoning and continuing.");
                nav_result_ = BT::NodeStatus::FAILURE;
            }
        };

    goal_done_ = false;
    nav_client_->async_send_goal(goal_msg, send_goal_options);

    RCLCPP_INFO(node_->get_logger(), "Sending Nav2 goal to (%.2f, %.2f)", 
                current_goal_.pose.position.x, current_goal_.pose.position.y);

    publishFrontiers(global_frame);

    return BT::NodeStatus::RUNNING;
}

void RoomExplorerAction::onHalted()
{
    RCLCPP_INFO(node_->get_logger(), "RoomExplorer node halted. Canceling Nav2 goal.");
    if (current_goal_handle_) {
        nav_client_->async_cancel_goal(current_goal_handle_);
    }
}

// Add to the bottom of room_explorer.cpp
void RoomExplorerAction::publishFrontiers(const std::string& frame_id)
{
    visualization_msgs::msg::Marker msg;
    msg.header.frame_id = frame_id; 
    msg.header.stamp = node_->get_clock()->now();
    msg.ns = "dfs_stack";
    msg.id = 0;
    
    if (frontier_stack_.empty()) {
        msg.action = visualization_msgs::msg::Marker::DELETEALL;
        pub_frontier_markers_->publish(msg);
        return;
    }

    msg.type = visualization_msgs::msg::Marker::LINE_LIST;
    msg.action = visualization_msgs::msg::Marker::ADD;
    msg.scale.x = 0.1; // Line width

    // Bright Green
    msg.color.r = 0.0; msg.color.g = 1.0; msg.color.b = 0.0; msg.color.a = 1.0;

    for (const auto& fg : frontier_stack_) {
        geometry_msgs::msg::Point p1, p2;
        p1.x = fg.door.p1.x; p1.y = fg.door.p1.y; p1.z = 0.15; // Raised slightly above ground
        p2.x = fg.door.p2.x; p2.y = fg.door.p2.y; p2.z = 0.15;
        msg.points.push_back(p1);
        msg.points.push_back(p2);
    }

    pub_frontier_markers_->publish(msg);
}

} // namespace go2_control_cpp

BT_REGISTER_NODES(factory)
{ 
  factory.registerBuilder<go2_control_cpp::RoomExplorerAction>(
    "RoomExplorerAction",
    [](auto & name, auto & config) {
      return std::make_unique<go2_control_cpp::RoomExplorerAction>(name, config);
    });
}