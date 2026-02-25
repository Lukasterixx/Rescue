// src/walk_bt_node.cpp
#include "go2_control_cpp/walk_bt_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/decorators/delay_node.h>

#include "go2_control_cpp/map2d_saver_node.hpp"
#include "go2_control_cpp/nav2_ready.hpp"
#include "go2_control_cpp/run_once.hpp"

#include "go2_control_cpp/topological_mapper.hpp"
#include "go2_control_cpp/distance_controller.hpp"
#include "go2_control_cpp/room_explorer.hpp"


using namespace std::chrono_literals;

namespace go2_control_cpp
{

WalkBTNode::WalkBTNode()
: Node("walk_bt_node")
{

}

void WalkBTNode::init(SharedPtr self)
{
  // 1) Register your custom BehaviorTree node builders
  
  factory_.registerBuilder<Nav2Ready>(
    "Nav2Ready",
    [](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<Nav2Ready>(name, config);
    });
  factory_.registerBuilder<RunOnce>(
    "RunOnce",
    [](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<RunOnce>(name, config);
    });
  factory_.registerBuilder<DistanceTickController>(
    "DistanceTickController",
    [](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<DistanceTickController>(name, config);
    });

  // register our Map2DSaver so the BT factory can create it
  this->declare_parameter<std::string>("maps_dir", "maps");
  this->get_parameter("maps_dir", maps_dir_);  // store it in a member of WalkBTNode

  factory_.registerBuilder<Map2DSaver>(
    "Map2DSaver",
    [](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<Map2DSaver>(name, config);
    });

  factory_.registerBuilder<TopologicalMapperAction>(
    "TopologicalMapperAction",
    [](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<TopologicalMapperAction>(name, config);
    });
  factory_.registerBuilder<RoomExplorerAction>(
    "RoomExplorerAction",
    [](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<RoomExplorerAction>(name, config);
    });

  // 2) Create a Blackboard, inject our ROS Node handle, then load the XML
  auto blackboard = BT::Blackboard::create();
  // store the shared_ptr<WalkBTNode> under the key "node"
  blackboard->set<rclcpp::Node::SharedPtr>("node", self);

  auto pkg_share = ament_index_cpp::get_package_share_directory("go2_control_cpp");
  auto xml_path  = pkg_share + "/go2_tree.xml";
  RCLCPP_INFO(get_logger(), "Loading BT from: %s", xml_path.c_str());
  // pass the Blackboard into the create call
  tree_ = factory_.createTreeFromFile(xml_path, blackboard);

  // 3) Start ticking the tree at 10 Hz
  timer_ = this->create_wall_timer(
    100ms,
    [this, self]() {
      tree_.rootNode()->executeTick();
    });
}

}  // namespace go2_control_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  // Create as shared_ptr so init() can capture it
  auto node = std::make_shared<go2_control_cpp::WalkBTNode>();
  node->init(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
