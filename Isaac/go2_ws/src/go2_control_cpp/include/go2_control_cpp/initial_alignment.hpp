#ifndef GO2_CONTROL_CPP_INITIAL_ALIGNMENT_HPP
#define GO2_CONTROL_CPP_INITIAL_ALIGNMENT_HPP

#include <string>
#include <memory>

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace go2_control_cpp
{

// Bundles translation and rotation into one port
struct Alignment
{
  Eigen::Vector3f translation;
  Eigen::Quaternionf rotation;
};

class InitialAlignment : public BT::AsyncActionNode
{
public:
  InitialAlignment(const std::string & name,
                   const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  // Called every tree tick until SUCCESS/FAILURE
  BT::NodeStatus tick() override;
  // Called if the node is halted externally
  void halt() override;

private:
  // One‐shot callback for the first cloud
  void cloudCb(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  // ROS node handle
  rclcpp::Node::SharedPtr node_;
  // Subscription for the first incoming scan
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;

  // Flags for state machine
  bool received_;
  bool processed_;

  // Filtered first scan
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_;

  // Where to save/load the scan
  std::string save_path_;
  
};

}  // namespace go2_control_cpp

#endif  // GO2_CONTROL_CPP_INITIAL_ALIGNMENT_HPP
