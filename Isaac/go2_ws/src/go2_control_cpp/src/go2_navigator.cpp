// File: src/go2_navigator.cpp

#include <memory>
#include <string>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "go2_control_navigator/go2_navigator.hpp"


namespace go2_control_navigator
{

bool Go2Navigator::configure(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node,
  std::shared_ptr<nav2_util::OdomSmoother> odom_smoother)
{


  auto node = parent_node.lock();

  if (!node->has_parameter("goal_blackboard_id")) {
    node->declare_parameter("goal_blackboard_id", std::string("goal"));
  }

  goal_blackboard_id_ = node->get_parameter("goal_blackboard_id").as_string();

  if (!node->has_parameter("path_blackboard_id")) {
    node->declare_parameter("path_blackboard_id", std::string("path"));
  }
  path_blackboard_id_ =
    node->get_parameter("path_blackboard_id").as_string();

  
  // for goal completion
  retick_pub_ = node->create_publisher<std_msgs::msg::Empty>("/retick", 1);


  // Odometry smoother object for getting current speed
  odom_smoother_ = odom_smoother;

  self_client_ = rclcpp_action::create_client<ActionT>(node, getName());

  goal_sub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
    "goal_pose",
    rclcpp::QoS(10),
    std::bind(&Go2Navigator::onGoalPoseReceived, this, std::placeholders::_1));

  RCLCPP_INFO(logger_, "Go2Navigator configured");

  return true;
}

std::string Go2Navigator::getDefaultBTFilepath(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node)
{
  auto node = parent_node.lock();
  std::string xml;
  auto pkg_share =
        ament_index_cpp::get_package_share_directory("go2_control_cpp");

  if (!node->has_parameter("bt_tree_pose")) {
    node->declare_parameter<std::string>(
      "bt_tree_pose",
      pkg_share + "/minimal_nav_tree.xml");
  }
  node->get_parameter("bt_tree_pose", xml);
  return pkg_share + "/" + xml;
}

bool Go2Navigator::cleanup()
{
  return true;
}

bool Go2Navigator::goalReceived(
  typename ActionT::Goal::ConstSharedPtr goal)
{
  RCLCPP_INFO(logger_, "Go2Navigator::goalReceived() was called!");
  
  // Choose the requested tree or fall back to default
  auto bt_file = goal->behavior_tree.empty()
    ? bt_action_server_->getDefaultBTFilename()
    : goal->behavior_tree;

  RCLCPP_INFO(logger_, "Go2Navigator: loading Behavior Tree XML from file: %s", bt_file.c_str());

  if (!bt_action_server_->loadBehaviorTree(bt_file)) {
    RCLCPP_ERROR(
      logger_, "BT file not found: %s. Navigation canceled.",
      bt_file.c_str());
    return false;
  }

  initializeGoalPose(goal);
  return true;
}

void Go2Navigator::goalCompleted(
  typename ActionT::Result::SharedPtr /*result*/,
  const nav2_behavior_tree::BtStatus /*status*/)
{
  // When the navigation BT finishes (successfully or failed), trigger the retick
  if (retick_pub_) {
    RCLCPP_INFO(logger_, "Goal completed: Publishing /retick to DistanceTickController");
    retick_pub_->publish(std_msgs::msg::Empty());
  }
}

void Go2Navigator::onLoop()
{
  // no feedback for minimalism
}

void
Go2Navigator::onPreempt(ActionT::Goal::ConstSharedPtr goal)
{
  RCLCPP_INFO(logger_, "Received goal preemption request");

  if (goal->behavior_tree == bt_action_server_->getCurrentBTFilename() ||
    (goal->behavior_tree.empty() &&
    bt_action_server_->getCurrentBTFilename() == bt_action_server_->getDefaultBTFilename()))
  {
    // if pending goal requests the same BT as the current goal, accept the pending goal
    // if pending goal has an empty behavior_tree field, it requests the default BT file
    // accept the pending goal if the current goal is running the default BT file
    initializeGoalPose(bt_action_server_->acceptPendingGoal());
  } else {
    RCLCPP_WARN(
      logger_,
      "Preemption request was rejected since the requested BT XML file is not the same "
      "as the one that the current goal is executing. Preemption with a new BT is invalid "
      "since it would require cancellation of the previous goal instead of true preemption."
      "\nCancel the current goal and send a new action request if you want to use a "
      "different BT XML file. For now, continuing to track the last goal until completion.");
    bt_action_server_->terminatePendingGoal();
  }
}


void Go2Navigator::initializeGoalPose(
  typename ActionT::Goal::ConstSharedPtr goal)
{
  auto bb = bt_action_server_->getBlackboard();
  // inject the PoseStamped goal under “goal” for ComputePathToPose
  bb->set<geometry_msgs::msg::PoseStamped>("goal", goal->pose);
}

void Go2Navigator::onGoalPoseReceived(const geometry_msgs::msg::PoseStamped::SharedPtr pose)
{
  RCLCPP_INFO(logger_, "goal_pose topic → sending NavigateToPose goal");
  ActionT::Goal goal;
  goal.pose = *pose;
  self_client_->async_send_goal(goal);
}

}  // namespace go2_control_navigator

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  go2_control_navigator::Go2Navigator,
  backported_bt_navigator::NavigatorBase)

  // PLUGINLIB_EXPORT_CLASS(
  //  opennav_coverage_navigator::CoverageNavigator,
  //  backported_bt_navigator::NavigatorBase)

//   <class_libraries>
// 	<library path="opennav_coverage_navigator">
// 	  <class
// 	  	name="opennav_coverage_navigator/CoverageNavigator"
// 	  	type="opennav_coverage_navigator::CoverageNavigator"
// 	  	base_class_type="backported_bt_navigator::NavigatorBase">
// 	    <description>Coverage Navigator</description>
// 	  </class>
// 	</library>
// </class_libraries>