#include "go2_control_cpp/map2d_saver_node.hpp"

#include <behaviortree_cpp_v3/tree_node.h>
#include <filesystem>
#include "ament_index_cpp/get_package_share_directory.hpp"


namespace go2_control_cpp
{

Map2DSaver::Map2DSaver(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & config)
: BT::ActionNodeBase(xml_tag_name, config)
{
  // pull the node pointer off the blackboard:
  node_ = this->config().blackboard
    ->get<rclcpp::Node::SharedPtr>("node");

  // std::string maps_dir = "/home/lukas/P2Dingo/Isaac/go2_ws/src/go2_control_cpp/maps";
  node_->get_parameter("maps_dir", maps_dir);

  // Resolve relative maps_dir against go2_control_cpp share dir
  if (!maps_dir.empty() && maps_dir[0] != '/') {
    auto pkg_share =
        ament_index_cpp::get_package_share_directory("go2_control_cpp");
    maps_dir = pkg_share + "/" + maps_dir;
  }

  map_output_base_name_ = maps_dir + "/map2d";


  // Read thresholds for free/occupied
  node_->declare_parameter<double>("free_thresh", free_thresh_);
  node_->declare_parameter<double>("occupied_thresh", occupied_thresh_);
  node_->get_parameter("free_thresh", free_thresh_);
  node_->get_parameter("occupied_thresh", occupied_thresh_);

  // Subscribe to the latched /map2d topic
  rclcpp::QoS qos(1);
  qos.transient_local();
  qos.reliable();
  map_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map2d",
    qos,
    [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
      latest_map_ = *msg;
      have_map_   = true;
    });
}

BT::NodeStatus Map2DSaver::tick()
{
  if (!have_map_) {
    RCLCPP_WARN(node_->get_logger(),
                "Map2DSaver: no /map2d message received yet, retrying");
    return BT::NodeStatus::RUNNING;
  }

  // Prepare save parameters
  nav2_map_server::SaveParameters params;
  params.map_file_name   = map_output_base_name_;
  params.image_format    = "pgm";
  params.free_thresh     = free_thresh_;
  params.occupied_thresh = occupied_thresh_;
  params.mode            = nav2_map_server::MapMode::Trinary;

  // Delegate to Nav2’s map saver
  if (!nav2_map_server::saveMapToFile(latest_map_, params)) {
    RCLCPP_ERROR(node_->get_logger(),
                 "Map2DSaver: saveMapToFile failed for '%s'",
                 params.map_file_name.c_str());
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(node_->get_logger(),
              "Map2DSaver: wrote '%s.pgm' & '%s.yaml'",
              params.map_file_name.c_str(),
              params.map_file_name.c_str());
  return BT::NodeStatus::SUCCESS;
}

}  // namespace go2_control_cpp

BT_REGISTER_NODES(factory)
{ 
  // Map2D saver
  factory.registerBuilder<go2_control_cpp::Map2DSaver>(
    "Map2DSaver",
    [](auto & name, auto & config) {
      return std::make_unique<go2_control_cpp::Map2DSaver>(name, config);
    });
}