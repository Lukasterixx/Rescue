#include "go2_control_cpp/return_to_base.hpp"
#include <behaviortree_cpp_v3/bt_factory.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

namespace go2_control_cpp
{

ReturnToBaseAction::ReturnToBaseAction(const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config),
      node_(rclcpp::Node::make_shared("return_to_base_node")),
      waypoints_active_(false),
      return_goal_active_(false),
      current_state_(ReturnState::NORMAL_OPERATION),
      target_yaw_(0.0)
{
    // 1. Initialize Action Clients
    // Note: Ensure "navigate_through_poses" matches the name in Go2NavigatorPoses if you have one.
    client_through_poses_ = rclcpp_action::create_client<nav2_msgs::action::NavigateThroughPoses>(
        node_, "navigate_through_poses");

    // --- FIX: Client name must match Go2Navigator::getName() ("go2_navigator") ---
    client_to_pose_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
        node_, "go2_navigator"); 
    // -----------------------------------------------------------------------------

    // 2. Subscribe to Waypoints (to check if ThroughPoses is active)
    sub_waypoints_ = node_->create_subscription<geometry_msgs::msg::PoseArray>(
        "/waypoints_farm", 10,
        [this](const geometry_msgs::msg::PoseArray::SharedPtr msg) {
            waypoints_active_ = !msg->poses.empty();
        });

    // NEW: Initialize Waypoints Publisher to clear RViz
    pub_waypoints_ = node_->create_publisher<geometry_msgs::msg::PoseArray>(
        "/waypoints_farm", rclcpp::QoS(10).transient_local());

    // 3. Trigger Return
    sub_return_ = node_->create_subscription<std_msgs::msg::Empty>(
        "/return_to_base", 10,
        [this](const std_msgs::msg::Empty::SharedPtr) {
            if (current_state_ == ReturnState::NORMAL_OPERATION) {
                
                // Stop whatever Nav2 is doing currently
                performNav2Reset();

                // Start 180 turn
                double current_yaw = getCurrentYaw();
                target_yaw_ = normalizeAngle(current_yaw + M_PI);
                
                RCLCPP_INFO(node_->get_logger(), "Return request. Reset Nav2. Starting 180 turn.");
                current_state_ = ReturnState::ROTATING;
            }
        });

    // 4. Trigger Restart
    sub_restart_ = node_->create_subscription<std_msgs::msg::Empty>(
        "/begin_routine_inspection", 10,
        [this](const std_msgs::msg::Empty::SharedPtr) {
            if (current_state_ == ReturnState::DOCKED) {
                RCLCPP_INFO(node_->get_logger(), "Inspection start received. Switching to NORMAL_OPERATION.");
                current_state_ = ReturnState::NORMAL_OPERATION;
            }
        });

    // 5. Publishers
    pub_goal_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
    pub_vel_ = node_->create_publisher<geometry_msgs::msg::Twist>("/robot0/cmd_vel", 10);

    // 6. TF & Executor
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    exec_ = std::thread([this]() { executor_->spin(); });
}

ReturnToBaseAction::~ReturnToBaseAction()
{
    if (executor_) executor_->cancel();
    if (exec_.joinable()) exec_.join();
}

BT::PortsList ReturnToBaseAction::providedPorts()
{
    return { 
        BT::InputPort<double>("xy_tolerance", 0.3, "Distance in meters to consider 'arrived'"),
        BT::InputPort<double>("rot_speed", 1.0, "Rotation speed in rad/s"),
        BT::InputPort<std::string>("robot_frame", "livox_frame", "Frame attached to robot"),
        BT::InputPort<std::string>("global_frame", "odom", "Global frame (origin)"),
        BT::InputPort<std::string>("cmd_vel_topic", "/robot0/cmd_vel", "Topic for velocity commands")
    };
}

BT::NodeStatus ReturnToBaseAction::onStart()
{
    if (!getInput("xy_tolerance", tolerance_)) tolerance_ = 0.3;
    if (!getInput("rot_speed", rot_speed_)) rot_speed_ = 1.0;
    getInput("robot_frame", robot_frame_);
    getInput("global_frame", global_frame_);
    
    std::string topic;
    if (getInput("cmd_vel_topic", topic) && topic != cmd_vel_topic_) {
        cmd_vel_topic_ = topic;
        pub_vel_ = node_->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    }
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ReturnToBaseAction::onRunning()
{
    switch (current_state_)
    {
    case ReturnState::NORMAL_OPERATION:
        return BT::NodeStatus::SUCCESS;

    case ReturnState::ROTATING: {
        double current_yaw = getCurrentYaw();
        double error = normalizeAngle(target_yaw_ - current_yaw);

        if (std::abs(error) < 0.1) {
            sendVelocity(0.0);
            RCLCPP_INFO(node_->get_logger(), "Rotation complete. Sending Action Goal to Base.");
            
            // Send Goal via Action Client (Robust)
            sendReturnGoalViaAction();
            
            current_state_ = ReturnState::RETURNING;
        } else {
            double vel = (error > 0) ? rot_speed_ : -rot_speed_;
            sendVelocity(vel);
        }
        return BT::NodeStatus::RUNNING;
    }

    case ReturnState::RETURNING:
        // 1. Manual check for arrival (TF)
        if (hasReachedOrigin()) {
            RCLCPP_INFO(node_->get_logger(), "Arrived at origin. Cancelling Nav2.");
            cancelNav2Action();
            current_state_ = ReturnState::DOCKED;
            return_goal_active_ = false;
            return BT::NodeStatus::FAILURE;
        }

        // 2. Check if Action Client Aborted (Safety Watchdog)
        if (!return_goal_active_) {
            RCLCPP_ERROR(node_->get_logger(), "Nav2 Action Aborted! (Planner failed?). Returning SUCCESS to stop.");
            // Reset to normal operation so we don't hang in a loop
            current_state_ = ReturnState::NORMAL_OPERATION; 
            return BT::NodeStatus::SUCCESS; 
        }

        return BT::NodeStatus::RUNNING;

    case ReturnState::DOCKED:
        return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
}

void ReturnToBaseAction::onHalted()
{
    if (current_state_ == ReturnState::ROTATING) {
        sendVelocity(0.0);
    }
    RCLCPP_INFO(node_->get_logger(), "ReturnToBaseAction halted");
}

void ReturnToBaseAction::performNav2Reset()
{
    if (waypoints_active_) {
        RCLCPP_INFO(node_->get_logger(), "Waypoints active: Cancelling ThroughPoses.");
        cancelWaypointsNavigation();
    } else {
        RCLCPP_INFO(node_->get_logger(), "Standard Nav: Cancelling Go2Navigator.");
        // OLD: resetNav2WithForwardGoal();
        // NEW: Explicit Cancel
        cancelNav2Action();
    }
}

void ReturnToBaseAction::cancelWaypointsNavigation()
{
    // 1. Send Explicit Cancel Request to Nav2
    if (client_through_poses_->action_server_is_ready()) {
        client_through_poses_->async_cancel_all_goals();
        RCLCPP_INFO(node_->get_logger(), "Sent CancelAllGoals to NavigateThroughPoses.");
    } else {
        RCLCPP_WARN(node_->get_logger(), "NavigateThroughPoses server not ready, could not cancel.");
    }

    // 2. Force Stop the Robot (Safety)
    // This overrides any lingering commands from the previous controller
    sendVelocity(0.0);

    // 3. Clear RViz Visualization
    // Since we are cancelling via the client, the Navigator's onPreempt won't run.
    // We must clear the markers ourselves.
    geometry_msgs::msg::PoseArray empty_msg;
    empty_msg.header.stamp = node_->get_clock()->now();
    empty_msg.header.frame_id = global_frame_; // Usually "map" or "odom"
    pub_waypoints_->publish(empty_msg);
    
    RCLCPP_INFO(node_->get_logger(), "Cleared /waypoints_farm visualization and stopped robot.");
}

void ReturnToBaseAction::resetNav2WithForwardGoal()
{
    try {
        geometry_msgs::msg::TransformStamped t;
        t = tf_buffer_->lookupTransform(global_frame_, robot_frame_, tf2::TimePointZero);
        double yaw = tf2::getYaw(t.transform.rotation);

        double forward_dist = 1.0;
        double target_x = t.transform.translation.x + (forward_dist * std::cos(yaw));
        double target_y = t.transform.translation.y + (forward_dist * std::sin(yaw));

        geometry_msgs::msg::PoseStamped goal;
        goal.header.stamp = node_->get_clock()->now();
        goal.header.frame_id = global_frame_;
        goal.pose.position.x = target_x;
        goal.pose.position.y = target_y;
        goal.pose.position.z = t.transform.translation.z;
        goal.pose.orientation = t.transform.rotation;

        pub_goal_->publish(goal);
    }
    catch (const tf2::TransformException & ex) {}
}

void ReturnToBaseAction::stopNav2WithCurrentPose()
{
    try {
        geometry_msgs::msg::TransformStamped t;
        t = tf_buffer_->lookupTransform(global_frame_, robot_frame_, tf2::TimePointZero);

        geometry_msgs::msg::PoseStamped goal;
        goal.header.stamp = node_->get_clock()->now();
        goal.header.frame_id = global_frame_;
        goal.pose.position.x = t.transform.translation.x;
        goal.pose.position.y = t.transform.translation.y;
        goal.pose.position.z = t.transform.translation.z;
        goal.pose.orientation = t.transform.rotation;

        pub_goal_->publish(goal);
    }
    catch (const tf2::TransformException & ex) {}
}

void ReturnToBaseAction::sendReturnGoalViaAction()
{
    // Wait for "go2_navigator"
    if (!client_to_pose_->wait_for_action_server(std::chrono::milliseconds(2000))) {
        RCLCPP_ERROR(node_->get_logger(), "Action server 'go2_navigator' not available!");
        return;
    }

    auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
    goal_msg.pose.header.stamp = node_->get_clock()->now();
    goal_msg.pose.header.frame_id = global_frame_;
    goal_msg.pose.pose.position.x = 0.0;
    goal_msg.pose.pose.position.y = 0.0;
    goal_msg.pose.pose.position.z = 0.0;
    goal_msg.pose.pose.orientation.w = 1.0;

    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    
    send_goal_options.result_callback = 
        [this](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result) {
            
            return_goal_active_ = false; // Mark done

            switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    RCLCPP_INFO(node_->get_logger(), "Nav2: Goal Reached.");
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_ERROR(node_->get_logger(), "Nav2: Goal was ABORTED.");
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_WARN(node_->get_logger(), "Nav2: Goal was CANCELED.");
                    break;
                default:
                    RCLCPP_ERROR(node_->get_logger(), "Nav2: Unknown result code.");
                    break;
            }
        };

    return_goal_active_ = true;
    client_to_pose_->async_send_goal(goal_msg, send_goal_options);
    RCLCPP_INFO(node_->get_logger(), "Sent Return Goal via Action Client.");
}

void ReturnToBaseAction::sendVelocity(double angular_z)
{
    geometry_msgs::msg::Twist msg;
    msg.linear.x = 0.0;
    msg.angular.z = angular_z;
    pub_vel_->publish(msg);
}

double ReturnToBaseAction::getCurrentYaw()
{
    try {
        geometry_msgs::msg::TransformStamped t;
        t = tf_buffer_->lookupTransform(global_frame_, robot_frame_, tf2::TimePointZero);
        return tf2::getYaw(t.transform.rotation);
    }
    catch (const tf2::TransformException & ex) {
        return 0.0;
    }
}

double ReturnToBaseAction::normalizeAngle(double angle)
{
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

bool ReturnToBaseAction::hasReachedOrigin()
{
    try {
        geometry_msgs::msg::TransformStamped t;
        t = tf_buffer_->lookupTransform(global_frame_, robot_frame_, tf2::TimePointZero);
        double dist = std::sqrt(std::pow(t.transform.translation.x, 2) + std::pow(t.transform.translation.y, 2));
        return dist < tolerance_;
    }
    catch (const tf2::TransformException &) {
        return false;
    }
}

void ReturnToBaseAction::cancelNav2Action()
{
    // Check if server is available
    if (!client_to_pose_->action_server_is_ready()) {
        RCLCPP_WARN(node_->get_logger(), "Cannot cancel: 'go2_navigator' server not ready.");
        return;
    }

    // This sends a formal Cancel Request to the Action Server
    client_to_pose_->async_cancel_all_goals();
    RCLCPP_INFO(node_->get_logger(), "Sent CancelAllGoals request to Nav2.");
}

} // namespace go2_control_cpp

BT_REGISTER_NODES(factory)
{ 
  factory.registerBuilder<go2_control_cpp::ReturnToBaseAction>(
    "ReturnToBaseAction",
    [](auto & name, auto & config) {
      return std::make_unique<go2_control_cpp::ReturnToBaseAction>(name, config);
    });
}