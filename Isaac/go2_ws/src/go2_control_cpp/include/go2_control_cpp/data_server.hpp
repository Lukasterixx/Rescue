// data_server.hpp
#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <mutex>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Eigen/Dense>
#include <cmath>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <nlohmann/json.hpp>
#include <fstream>
#include <iomanip>

#include <tf2_ros/transform_broadcaster.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "go2_control_cpp/common_types.hpp"

namespace go2_control_cpp
{

// Individual panels comprising the Unit
struct Unit
{
    int row;
    int col;
    bool first_col_right;
    cv::Point2f center;
    std::array<cv::Point2f, 4> corners; // top left, top right, bottom left, bottom right
    int direction; // 0 to 360 true bearing
    bool is_first;
};

// Table containing a 2D matrix of Units
struct Table 
{
    int row;
    cv::Point2f center;
    std::array<cv::Point2f, 4> corners; // top left, top right, bottom left, bottom right
    int direction; // 0 to 360 true bearing
    bool is_first;
    bool is_half;
    double angle_rad = 0.0; // <--- NEW: Stores the global angle of the row

    // 2D matrix of Units: units[row][col]
    std::vector<std::vector<Unit>> units;
};

class DataServer : public BT::SyncActionNode
{
public:
  DataServer(const std::string& xml_tag_name, const BT::NodeConfiguration& config);
  
  static BT::PortsList providedPorts()
  {
      return {
          BT::InputPort<std::vector<Panel>>("panels"),
          BT::InputPort<double>("average_angle"),
          BT::OutputPort<CurrentPanel>("current_panel", "Struct containing ID, geometry, and scan settings for the active panel")
    };
  }

  // AsyncActionNode overrides
  BT::NodeStatus tick() override;

private:
  // ROS node handle
  std::shared_ptr<rclcpp::Node> node_;

  // Subscriptions
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_cloud_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr waypoint_sub_;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr units_marker_pub_;

  // Stored data
  nav_msgs::msg::OccupancyGrid::SharedPtr last_map_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr        last_cloud_;
  geometry_msgs::msg::PoseStamped::SharedPtr last_pose_;
  geometry_msgs::msg::PoseArray::SharedPtr   last_waypoints_;
  std::vector<Table> tables_;


  std::mutex mutex_;

  // Parameters
  double table_height_m_;
  double table_tilt_deg_;
  double unit_height_mm_;
  double unit_width_mm_;
  double unit_sep_mm_;

  // TF broadcaster for the "units" frame
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr                  tf_timer_;
  double                                         avg_angle_deg_{0.0};
  void BroadcastUnitsFrame(double yaw_deg);

  std::shared_ptr<tf2_ros::Buffer>          tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr camera_pub_;

  bool updatePoseFromTF();

  // Callbacks
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void waypointsCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);

  // Compute panel dimensions from cropped pointcloud
  bool UpdatePanelDimensions();
  bool LoadPanels(std::vector<Table>& tables);
  bool GenerateJSON();

  /// Draw every Unit as a LINE_STRIP marker in “map2d” frame
  void PublishUnitMarkers(const std::vector<Table>& tables);

  bool exploration_mode_{false};


};

} // namespace go2_control_cpp
