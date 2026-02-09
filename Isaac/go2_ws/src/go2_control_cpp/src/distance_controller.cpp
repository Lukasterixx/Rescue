#include "go2_control_cpp/distance_controller.hpp"
#include <behaviortree_cpp_v3/bt_factory.h>

BT_REGISTER_NODES(factory)
{
  factory.registerBuilder<go2_control_cpp::DistanceTickController>(
    "DistanceTickController",
    [](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<go2_control_cpp::DistanceTickController>(name, config);
    }
  );
}