#ifndef GO2_CONTROL_NAVIGATOR__GO2_NAVIGATOR_POSES_HPP_
#define GO2_CONTROL_NAVIGATOR__GO2_NAVIGATOR_POSES_HPP_

#include <string>
#include <vector>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "go2_control_navigator/behavior_tree_navigator.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/odometry_utils.hpp"
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

namespace go2_control_navigator
{

class NavigateThroughPosesNavigator
  : public backported_bt_navigator::BehaviorTreeNavigator<nav2_msgs::action::NavigateThroughPoses>
{
public:
  using ActionT = nav2_msgs::action::NavigateThroughPoses;
  typedef std::vector<geometry_msgs::msg::PoseStamped> Goals;

  NavigateThroughPosesNavigator()
  : BehaviorTreeNavigator() {}

  bool configure(
    rclcpp_lifecycle::LifecycleNode::WeakPtr node,
    std::shared_ptr<nav2_util::OdomSmoother> odom_smoother) override;

  std::string getName() override {return std::string("navigate_through_poses");}

  std::string getDefaultBTFilepath(rclcpp_lifecycle::LifecycleNode::WeakPtr node) override;

protected:
  bool goalReceived(ActionT::Goal::ConstSharedPtr goal) override;
  void onLoop() override;
  void onPreempt(ActionT::Goal::ConstSharedPtr goal) override;
  void goalCompleted(
    typename ActionT::Result::SharedPtr result,
    const nav2_behavior_tree::BtStatus final_bt_status) override;

  void initializeGoalPoses(ActionT::Goal::ConstSharedPtr goal);

  // Renamed to reflect new logic
  std::vector<geometry_msgs::msg::PoseStamped> updateGoalWindow();

  rclcpp::Time start_time_;
  std::string goals_blackboard_id_;
  std::string path_blackboard_id_;
  
  double pose_reached_threshold_{0.5}; // Default 0.5m
  int goal_window_size_{5};            // Sliding window size

  // Odometry smoother object
  std::shared_ptr<nav2_util::OdomSmoother> odom_smoother_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr waypoints_pub_;

  // Master list of all goals
  Goals all_poses_;
  
  // Tracks the index of the *next* unreached waypoint in all_poses_
  size_t current_goal_index_{0};
};

}  // namespace go2_control_navigator

#endif  // GO2_CONTROL_NAVIGATOR__GO2_NAVIGATOR_POSES_HPP_