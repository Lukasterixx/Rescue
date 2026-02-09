// src/initial_alignment.cpp

#include <filesystem>
#include <limits>
#include <memory>
#include <string>

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/gicp.h>
#include <behaviortree_cpp_v3/bt_factory.h>


#include "go2_control_cpp/initial_alignment.hpp"

namespace go2_control_cpp
{

InitialAlignment::InitialAlignment(
    const std::string & name,
    const BT::NodeConfiguration & config)
: BT::AsyncActionNode(name, config),
  received_(false),
  processed_(false)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  node_->declare_parameter("save_path", std::string("/home/lukas/P2Dingo/Isaac/go2_ws/src/go2_control_cpp/maps/initial_scan.pcd"));
  node_->get_parameter("save_path", save_path_);

  sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/point_cloud2",
    rclcpp::SensorDataQoS(),
    std::bind(&InitialAlignment::cloudCb, this, std::placeholders::_1));
}

BT::PortsList InitialAlignment::providedPorts()
{
  return {
    BT::InputPort<double>("min_height", "filter out points with z < this"),
    BT::OutputPort<Alignment>("alignment", "computed translation + rotation"),
    BT::OutputPort<bool>("align_provided", "if file exists")
  };
}

BT::NodeStatus InitialAlignment::tick()
{
  if (!received_)
    return BT::NodeStatus::RUNNING;
  if (processed_)
    return BT::NodeStatus::SUCCESS;
  if (!filtered_) {
    RCLCPP_ERROR(node_->get_logger(), "[InitialAlignment] no filtered cloud");
    return BT::NodeStatus::FAILURE;
  }

  using PointT = pcl::PointXYZ;

  if (!std::filesystem::exists(save_path_))
  {
    setOutput("align_provided", false);

    if (pcl::io::savePCDFileBinary(save_path_, *filtered_) != 0) {
      RCLCPP_ERROR(node_->get_logger(), "[InitialAlignment] failed to save '%s'", save_path_.c_str());
      return BT::NodeStatus::FAILURE;
    }
    RCLCPP_INFO(node_->get_logger(), "[InitialAlignment] saved initial scan to '%s'", save_path_.c_str());
  }
  else
  {
    auto previous = std::make_shared<pcl::PointCloud<PointT>>();
    if (pcl::io::loadPCDFile(save_path_, *previous) < 0) {
      RCLCPP_ERROR(node_->get_logger(), "[InitialAlignment] failed to load '%s'", save_path_.c_str());
      return BT::NodeStatus::FAILURE;
    }

    pcl::GeneralizedIterativeClosestPoint<PointT, PointT> gicp;
    gicp.setInputSource(filtered_);
    gicp.setInputTarget(previous);
    gicp.align(*filtered_);
    if (!gicp.hasConverged()) {
      RCLCPP_ERROR(node_->get_logger(), "[InitialAlignment] GICP did not converge");
      return BT::NodeStatus::FAILURE;
    }

    Eigen::Matrix4f T = gicp.getFinalTransformation();
    Eigen::Quaternionf q(T.block<3,3>(0,0));

    Alignment result;
    result.translation = Eigen::Vector3f{T(0,3), T(1,3), T(2,3)};
    result.rotation    = q;
    setOutput("alignment", result);
    setOutput("align_provided", true);

    RCLCPP_INFO(node_->get_logger(),
                "[InitialAlignment] t=(%.3f,%.3f,%.3f) q=(%.3f,%.3f,%.3f,%.3f)",
                result.translation.x(),
                result.translation.y(),
                result.translation.z(),
                result.rotation.x(),
                result.rotation.y(),
                result.rotation.z(),
                result.rotation.w());
  }
  processed_ = true;
  return BT::NodeStatus::SUCCESS;
}

void InitialAlignment::halt()
{
  // nothing special to clean up, but could reset flags if needed
  received_ = false;
  processed_ = false;
}

void InitialAlignment::cloudCb(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (received_)
    return;

  using PointT = pcl::PointXYZ;
  auto cloud = std::make_shared<pcl::PointCloud<PointT>>();
  pcl::fromROSMsg(*msg, *cloud);

  double min_h = 0.0;
  if (!getInput("min_height", min_h)) {
    RCLCPP_ERROR(node_->get_logger(), "[InitialAlignment] missing min_height port");
    return;
  }

  pcl::PassThrough<PointT> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(min_h, std::numeric_limits<double>::max());

  filtered_ = std::make_shared<pcl::PointCloud<PointT>>();
  pass.filter(*filtered_);

  received_ = true;
}

}  // namespace go2_control_cpp

// Register node
BT_REGISTER_NODES(factory)
{ 
  factory.registerBuilder<go2_control_cpp::InitialAlignment>(
    "InitialAlignment",
    [](auto & name, auto & config) {
      return std::make_unique<go2_control_cpp::InitialAlignment>(name, config);
    });
}
