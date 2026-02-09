// File: include/go2_control_navigator/go2_navigator.hpp

#ifndef GO2_CONTROL_NAVIGATOR__GO2_NAVIGATOR_HPP_
#define GO2_CONTROL_NAVIGATOR__GO2_NAVIGATOR_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "go2_control_navigator/behavior_tree_navigator.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/odometry_utils.hpp"


namespace go2_control_navigator
{

/**
 * @class Go2Navigator
 * @brief A NavigatorBase plugin that loads a minimal BT (ComputePathToPose→FollowPath)
 *        and publishes its incoming goal into the BT blackboard under \"goal\".
 */
class Go2Navigator : public backported_bt_navigator::BehaviorTreeNavigator<nav2_msgs::action::NavigateToPose>
{
public:
  using ActionT = nav2_msgs::action::NavigateToPose;

  Go2Navigator() = default;
  ~Go2Navigator() override = default;
  std::string getName() override { return "go2_navigator"; }

  bool configure(
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node,
    std::shared_ptr<nav2_util::OdomSmoother> odom_smoother) override;

  std::string getDefaultBTFilepath(
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node) override;

  bool cleanup() override;

  bool goalReceived(
    typename ActionT::Goal::ConstSharedPtr goal) override;

  void goalCompleted(
    typename ActionT::Result::SharedPtr result,
    const nav2_behavior_tree::BtStatus status) override;

  void onLoop() override;

  void onPreempt(
    typename ActionT::Goal::ConstSharedPtr goal) override;

    /**
   * @brief A subscription and callback to handle the topic-based goal published
   * from rviz
   * @param pose Pose received via a topic
   */
  void onGoalPoseReceived(const geometry_msgs::msg::PoseStamped::SharedPtr pose);

  void initializeGoalPose(ActionT::Goal::ConstSharedPtr goal);

  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr retick_pub_;

    // Subscribe to PoseStamped goals on “goal_pose”
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp_action::Client<ActionT>::SharedPtr self_client_;

  std::string path_blackboard_id_;
  std::string goal_blackboard_id_;
  rclcpp::Time start_time_;

  // Odometry smoother object
  std::shared_ptr<nav2_util::OdomSmoother> odom_smoother_;
};

}  // namespace go2_control_navigator

#endif  // GO2_CONTROL_NAVIGATOR__GO2_NAVIGATOR_HPP_