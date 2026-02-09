// File: src/bt_go2.cpp FROM bt_navigator

#include "go2_control_cpp/bt_go2.hpp"

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_behavior_tree/bt_conversions.hpp"


using nav2_util::declare_parameter_if_not_declared;

namespace go2_control_cpp
{

BtGo2::BtGo2(rclcpp::NodeOptions options)
: nav2_util::LifecycleNode("bt_go2", "",
    options.automatically_declare_parameters_from_overrides(true)),
  class_loader_("go2_control_cpp", "backported_bt_navigator::NavigatorBase")
{
  RCLCPP_INFO(get_logger(), "Creating Go2 BT Navigator");

  // List of BT plugin libraries: all Nav2 nodes plus our go2_control_bt
  const std::vector<std::string> plugin_libs = {
    "nav2_compute_path_to_pose_action_bt_node",
    "nav2_compute_path_through_poses_action_bt_node",
    "nav2_smooth_path_action_bt_node",
    "nav2_follow_path_action_bt_node",
    "nav2_spin_action_bt_node",
    "nav2_wait_action_bt_node",
    "nav2_assisted_teleop_action_bt_node",
    "nav2_back_up_action_bt_node",
    "nav2_drive_on_heading_bt_node",
    "nav2_clear_costmap_service_bt_node",
    "nav2_is_stuck_condition_bt_node",
    "nav2_goal_reached_condition_bt_node",
    "nav2_initial_pose_received_condition_bt_node",
    "nav2_goal_updated_condition_bt_node",
    "nav2_globally_updated_goal_condition_bt_node",
    "nav2_is_path_valid_condition_bt_node",
    "nav2_are_error_codes_active_condition_bt_node",
    "nav2_would_a_controller_recovery_help_condition_bt_node",
    "nav2_would_a_planner_recovery_help_condition_bt_node",
    "nav2_would_a_smoother_recovery_help_condition_bt_node",
    "nav2_reinitialize_global_localization_service_bt_node",
    "nav2_rate_controller_bt_node",
    "nav2_distance_controller_bt_node",
    "nav2_speed_controller_bt_node",
    "nav2_truncate_path_action_bt_node",
    "nav2_truncate_path_local_action_bt_node",
    "nav2_goal_updater_node_bt_node",
    "nav2_recovery_node_bt_node",
    "nav2_pipeline_sequence_bt_node",
    "nav2_round_robin_node_bt_node",
    "nav2_transform_available_condition_bt_node",
    "nav2_time_expired_condition_bt_node",
    "nav2_path_expiring_timer_condition",
    "nav2_distance_traveled_condition_bt_node",
    "nav2_single_trigger_bt_node",
    "nav2_goal_updated_controller_bt_node",
    "nav2_is_battery_low_condition_bt_node",
    "nav2_navigate_through_poses_action_bt_node",
    "nav2_navigate_to_pose_action_bt_node",
    "nav2_remove_passed_goals_action_bt_node",
    "nav2_planner_selector_bt_node",
    "nav2_controller_selector_bt_node",
    "nav2_goal_checker_selector_bt_node",
    "nav2_controller_cancel_bt_node",
    "nav2_path_longer_on_approach_bt_node",
    "nav2_wait_cancel_bt_node",
    "nav2_spin_cancel_bt_node",
    "nav2_assisted_teleop_cancel_bt_node",
    "nav2_back_up_cancel_bt_node",
    "nav2_drive_on_heading_cancel_bt_node",
    "nav2_is_battery_charging_condition_bt_node",
    // finally, our custom GO2 nodes:
    "go2_control_bt_nodes"
  };

  // Declare parameters only if not overridden on the command line / in YAML
  declare_parameter_if_not_declared(
    this, "plugin_lib_names", rclcpp::ParameterValue(plugin_libs));
  declare_parameter_if_not_declared(
    this, "transform_tolerance", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(
    this, "global_frame", rclcpp::ParameterValue(std::string("odom")));
  declare_parameter_if_not_declared(
    this, "robot_base_frame", rclcpp::ParameterValue(std::string("livox_frame")));
  declare_parameter_if_not_declared(
    this, "odom_topic", rclcpp::ParameterValue(std::string("odom")));
}

BtGo2::~BtGo2() = default;

nav2_util::CallbackReturn
BtGo2::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring Go2 BT Navigator");

  // Set up TF buffer & listener
  tf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  auto timer_iface = std::make_shared<tf2_ros::CreateTimerROS>(
    get_node_base_interface(), get_node_timers_interface());
  tf_->setCreateTimerInterface(timer_iface);
  tf_->setUsingDedicatedThread(true);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_, this, false);

  // Read parameters
  global_frame_ = get_parameter("global_frame").as_string();
  robot_frame_ = get_parameter("robot_base_frame").as_string();
  transform_tolerance_ = get_parameter("transform_tolerance").as_double();
  odom_topic_ = get_parameter("odom_topic").as_string();

  // Plugin libraries to load
  auto plugin_lib_names = get_parameter("plugin_lib_names").as_string_array();

  // Prepare feedback utils
  backported_bt_navigator::FeedbackUtils feedback_utils;
  feedback_utils.tf = tf_;
  feedback_utils.global_frame = global_frame_;
  feedback_utils.robot_frame = robot_frame_;
  feedback_utils.transform_tolerance = transform_tolerance_;

  // Odometry smoother for velocity feedback
  auto node = shared_from_this();
  odom_smoother_ = std::make_shared<nav2_util::OdomSmoother>(node, 0.3, odom_topic_);

  // Which navigators to load (defaults to the two standard ones)
  const std::vector<std::string> default_ids   = {
    "navigate_to_pose", "navigate_through_poses"
  };
  const std::vector<std::string> default_types = {
    "nav2_bt_navigator/NavigateToPoseNavigator",
    "nav2_bt_navigator/NavigateThroughPosesNavigator"
  };

  declare_parameter_if_not_declared(
    node, "navigators", rclcpp::ParameterValue(default_ids));
  std::vector<std::string> navigator_ids;
  get_parameter("navigators", navigator_ids);

  // If user didn't override, declare each `<id>.plugin` param
  if (navigator_ids == default_ids) {
    for (size_t i = 0; i < default_ids.size(); ++i) {
      declare_parameter_if_not_declared(
        node,
        default_ids[i] + ".plugin",
        rclcpp::ParameterValue(default_types[i]));
    }
  }

  // Load & configure each navigator plugin
  for (auto & id : navigator_ids) {
    try {
      auto plugin_type =
        nav2_util::get_plugin_type_param(node, id);
      RCLCPP_INFO(
        get_logger(), "Creating navigator '%s' of type '%s'",
        id.c_str(), plugin_type.c_str());
      auto nav_inst = class_loader_.createUniqueInstance(plugin_type);
      if (!nav_inst->on_configure(
          node, plugin_lib_names,
          feedback_utils, &plugin_muxer_,
          odom_smoother_))
      {
        return nav2_util::CallbackReturn::FAILURE;
      }
      navigators_.push_back(std::move(nav_inst));
    } catch (const std::exception & ex) {
      RCLCPP_FATAL(
        get_logger(),
        "Failed to create navigator '%s': %s",
        id.c_str(), ex.what());
      return nav2_util::CallbackReturn::FAILURE;
    }
  }

  // advertise a latched “ready” topic so subscribers will get the message even if they connect late
  ready_pub_ = this->create_publisher<std_msgs::msg::Empty>(
    "/nav2_ready",
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
BtGo2::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating Go2 BT Navigator");

  // activate the publisher (needed for lifecycle‐aware publishers)
  ready_pub_->on_activate();

  // publish a single “you’re ready” message
  auto msg = std_msgs::msg::Empty();
  ready_pub_->publish(msg);

  for (auto & nav : navigators_) {
    if (!nav->on_activate()) {
      return nav2_util::CallbackReturn::FAILURE;
    }
  }
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
BtGo2::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating Go2 BT Navigator");
  for (auto & nav : navigators_) {
    if (!nav->on_deactivate()) {
      return nav2_util::CallbackReturn::FAILURE;
    }
  }
  destroyBond();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
BtGo2::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Cleaning up Go2 BT Navigator");
  tf_listener_.reset();
  tf_.reset();
  for (auto & nav : navigators_) {
    if (!nav->on_cleanup()) {
      return nav2_util::CallbackReturn::FAILURE;
    }
  }
  navigators_.clear();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
BtGo2::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down Go2 BT Navigator");
  return nav2_util::CallbackReturn::SUCCESS;
}

}  // namespace go2_control_cpp


#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(go2_control_cpp::BtGo2)

// RCLCPP_COMPONENTS_REGISTER_NODE(backported_bt_navigator::BtNavigator)