// File: src/initial_pose.cpp

#include "go2_control_cpp/initial_pose.hpp"

#include <cmath>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <tf2/LinearMath/Quaternion.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <Eigen/Dense>
#include <tf2/LinearMath/Matrix3x3.h>

namespace go2_control_cpp
{

InitialPose::InitialPose(
    const std::string & name,
    const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config),
    done_(false)
{
  // Grab our ports
  getInput("input_topic",            input_topic_);
  getInput("z_height",               z_height_);
  getInput("seg_distance_threshold", seg_distance_threshold_);
  getInput("seg_axis_tolerance_deg", seg_axis_tolerance_deg_);
  getInput("seg_max_iterations",     seg_max_iterations_);

  // Create a minimal ROS node just for this BT leaf
  node_ = rclcpp::Node::make_shared("floor_initial_pose_bt");
  pub_  = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/initial_pose", rclcpp::QoS(1));

  // Subscribe once to grab the very first scan
  sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&InitialPose::cloudCb, this, std::placeholders::_1));
}

void InitialPose::cloudCb(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (!done_ && !first_cloud_) {
    first_cloud_ = msg;
  }
}

BT::NodeStatus InitialPose::tick()
{
  RCLCPP_INFO(node_->get_logger(), "FloorInitialPose: starting floor detection");
  // If we already ran once, immediately succeed
  if (done_) {
    return BT::NodeStatus::SUCCESS;
  }

  // Spin once to receive a cloud
  rclcpp::spin_some(node_);
  if (!first_cloud_) {
    return BT::NodeStatus::RUNNING;
  }

  // Convert to PCL
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*first_cloud_, *pc);

  // RANSAC plane fit
  pcl::ModelCoefficients::Ptr coeffs(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(seg_distance_threshold_);
  seg.setAxis(Eigen::Vector3f(0, 0, 1));
  seg.setEpsAngle(seg_axis_tolerance_deg_ * M_PI / 180.0);
  seg.setMaxIterations(seg_max_iterations_);
  seg.setInputCloud(pc);
  seg.segment(*inliers, *coeffs);

  // Determine floor normal (default straight up)
  Eigen::Vector3f normal(0, 0, 1);
  if (coeffs->values.size() >= 3) {
    normal = Eigen::Vector3f(
      coeffs->values[0],
      coeffs->values[1],
      coeffs->values[2]
    ).normalized();
    if (normal.dot(Eigen::Vector3f(0,0,1)) < 0) {
      normal = -normal;
    }
  }

  // Compute quaternion rotating 'normal' -> Z axis
  Eigen::Vector3f axis = normal.cross(Eigen::Vector3f(0,0,1));
  double angle = std::acos(normal.dot(Eigen::Vector3f(0,0,1)));
  tf2::Quaternion q;
  if (axis.norm() < 1e-6 || std::abs(angle) < 1e-3) {
    q.setRPY(0,0,0);
  } else {
    axis.normalize();
    q.setRotation(tf2::Vector3(axis.x(), axis.y(), axis.z()), angle);
  }

   // --- NEW: print out the correction as roll/pitch/yaw ---
  {
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    RCLCPP_INFO(
      node_->get_logger(),
      "FloorInitialPose: applying correction RPY = [%.2f°, %.2f°, %.2f°] /////////////////////////////////////////////",
      roll * 180.0/M_PI,
      pitch * 180.0/M_PI,
      yaw * 180.0/M_PI
    );
  }

  // Publish the initial_pose
  geometry_msgs::msg::PoseStamped ip;
  ip.header = first_cloud_->header;
  ip.header.frame_id = "map";
  ip.pose.position.x = 0.0;
  ip.pose.position.y = 0.0;
  ip.pose.position.z = z_height_;
  ip.pose.orientation.x = q.x();
  ip.pose.orientation.y = q.y();
  ip.pose.orientation.z = q.z();
  ip.pose.orientation.w = q.w();
  pub_->publish(ip);

  done_ = true;
  return BT::NodeStatus::SUCCESS;
}

}  // namespace go2_control_cpp

// Register this node with BehaviorTree.CPP
BT_REGISTER_NODES(factory)
{
  factory.registerBuilder<go2_control_cpp::InitialPose>(
    "InitialPose",
    [](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<go2_control_cpp::InitialPose>(name, config);
    });
}
