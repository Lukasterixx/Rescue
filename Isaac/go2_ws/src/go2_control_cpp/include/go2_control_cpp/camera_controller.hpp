#ifndef GO2_CONTROL_CPP_CAMERA_CONTROLLER_HPP_
#define GO2_CONTROL_CPP_CAMERA_CONTROLLER_HPP_

#include <string>
#include <vector>
#include <mutex>
#include <cmath>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/action_node.h"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h" 
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "go2_control_cpp/common_types.hpp"


// Custom type specialization for BehaviorTree
namespace BT
{
    template <> inline go2_control_cpp::CurrentPanel convertFromString(StringView str)
    {
        (void)str;
        return {}; 
    }
}

namespace go2_control_cpp
{

class CameraController : public BT::StatefulActionNode
{
public:
  CameraController(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();

  // Lifecycle methods
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  // Callbacks and Helpers
  void waypointsCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);
  void PointCamera(double lat_dist, double height_m, double target_yaw);

  // Methods
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg); // NEW
  std::string DetermineScanDirection(double& out_lateral_dist);        // NEW
  
  // Updated to support string-based storage IDs (e.g., "1.1")
  void CheckPhotoTrigger(double target_dist_3d, const std::string& storage_id);
  
  // Helper to stop recording (and save JSON)
  void StopRecording(); 
  
  // Helper to start rosbag (and store start metadata)
  void StartRosBag(const std::string& storage_id, double initial_dist);

  // Helper for decimal versioning logic
  std::string GetNextAvailableFolder(int row_id);

  // ROS Interfaces
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr cam_pose_pub_;
  
  // Changed to String to support folder names like "1.1"
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr photo_req_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr video_req_pub_;
  
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // State
  nav_msgs::msg::OccupancyGrid::SharedPtr last_map_; // NEW
  std::mutex map_mutex_;                             // NEW

  // Subscriptions
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_; // NEW

  // Thread Safety
  std::mutex mutex_;
  geometry_msgs::msg::PoseArray::SharedPtr last_waypoints_;
  bool new_waypoints_available_ = false;

  // Logic State (Photo)
  double last_valid_height_ = 1.0;
  int current_row_id_ = -1;
  bool first_tick_ = true;
  double dist_accumulator_ = 0.0;
  
  // Anchor / Drift Logic State
  geometry_msgs::msg::TransformStamped anchor_pose_;
  double bb_lat_dist_anchor_ = 0.0;
  double cached_bb_dist_ = -1.0;
  geometry_msgs::msg::TransformStamped last_robot_pose_;

  // Video Logic State
  bool is_recording_ = false;
  double video_dist_accumulator_ = 0.0;
  int current_video_row_ = -1;
  int last_recorded_row_id_ = -1; 
  
  // Tracks the active folder name (e.g., "1", "1.1", "1.2")
  std::string current_storage_id_;

  // Metadata Storage
  rclcpp::Time recording_start_time_;
  double recorded_initial_dist_ = 0.0;

  // Constants
  const double CAMERA_Z_WORLD = 0.6;    
  const double HORIZ_APERTURE_MM = 7.68;
  const double FOCAL_LENGTH_MM = 13.0;
  std::string active_scan_dir_;
};

} // namespace go2_control_cpp

#endif // GO2_CONTROL_CPP_CAMERA_CONTROLLER_HPP_