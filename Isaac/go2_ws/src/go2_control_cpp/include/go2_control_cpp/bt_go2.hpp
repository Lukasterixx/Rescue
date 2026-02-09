// File: include/go2_control_navigator/bt_go2.hpp
// Copyright (c) 2025 Your Organization
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef GO2_CONTROL_NAVIGATOR__BT_GO2_HPP_
#define GO2_CONTROL_NAVIGATOR__BT_GO2_HPP_

#include <memory>
#include <string>
#include <vector>

#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"
#include "go2_control_navigator/behavior_tree_navigator.hpp"
#include "pluginlib/class_loader.hpp"
#include "std_msgs/msg/empty.hpp"


namespace go2_control_cpp
{
/**
 * @class go2_control_navigator::BtGo2
 * @brief A BehaviorTree.CPP–based navigator component that wraps Nav2’s
 *        compute/controller/BT nodes and your custom GO2 BT nodes into a
 *        lifecycle-managed ROS 2 component.
 */
class BtGo2 : public nav2_util::LifecycleNode    
{
public:
  /**
   * @brief Constructor for go2_control_navigator::BtGo2
   * @param options Node creation options (e.g. auto-parameter declaration)
   */
  explicit BtGo2(rclcpp::NodeOptions options = rclcpp::NodeOptions());

  /**
   * @brief Destructor for go2_control_navigator::BtGo2
   */
  ~BtGo2();

protected:
  /**
   * @brief Lifecycle configure hook: set up TF, odometry smoother, and load BT plugins & navigators
   */
  nav2_util::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Lifecycle activate hook: activate all loaded navigator instances
   */
  nav2_util::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Lifecycle deactivate hook: deactivate all loaded navigator instances
   */
  nav2_util::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Lifecycle cleanup hook: clean up TF, odometry smoother, and navigator instances
   */
  nav2_util::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Lifecycle shutdown hook
   */
  nav2_util::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & state) override;


  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Empty>::SharedPtr ready_pub_;

  pluginlib::ClassLoader<backported_bt_navigator::NavigatorBase> class_loader_;

  // Loaded navigator plugin instances
  std::vector<pluginlib::UniquePtr<backported_bt_navigator::NavigatorBase>> navigators_;

  // Internal multiplexer for combining plugin feedback
  backported_bt_navigator::NavigatorMuxer plugin_muxer_;

  // Odometry smoother for velocity feedback
  std::shared_ptr<nav2_util::OdomSmoother> odom_smoother_;

  // Frame and transform parameters
  std::string robot_frame_;
  std::string global_frame_;
  double transform_tolerance_;
  std::string odom_topic_;


  // TF buffer & listener for transform lookups
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace go2_control_cpp

#endif  // GO2_CONTROL_NAVIGATOR__BT_GO2_HPP_
