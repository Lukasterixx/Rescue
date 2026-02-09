#ifndef GO2_CONTROL_CPP__MAP2D_SAVER_NODE_HPP_
#define GO2_CONTROL_CPP__MAP2D_SAVER_NODE_HPP_

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav2_map_server/map_io.hpp>
#include <nav2_map_server/map_mode.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <string>
#include <memory>                 // for std::shared_ptr


namespace go2_control_cpp
{

class Map2DSaver : public BT::ActionNodeBase
{
public:
  Map2DSaver(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts() 
  { 
    return {
      BT::InputPort<std::shared_ptr<rclcpp::Node>>("node")
    }; 
  }

  BT::NodeStatus tick() override;
  void halt() override {}

private:
  std::shared_ptr<rclcpp::Node> node_;
  // subscriber to /map2d
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  nav_msgs::msg::OccupancyGrid  latest_map_;
  bool                          have_map_{false};

  // parameters for saving
  std::string                   map_output_base_name_;
  double                        free_thresh_;
  double                        occupied_thresh_;
  std::string maps_dir;
};

}  // namespace go2_control_cpp

#endif  // GO2_CONTROL_CPP__MAP2D_SAVER_NODE_HPP_
