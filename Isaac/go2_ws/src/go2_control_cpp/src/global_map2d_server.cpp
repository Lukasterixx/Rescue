#include "go2_control_cpp/global_map2d_server.hpp"

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <nav2_map_server/map_io.hpp>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "go2_control_cpp/path_planner.hpp"
#include <opencv2/opencv.hpp>

namespace go2_control_cpp
{

static inline void setUnknownToFree(nav_msgs::msg::OccupancyGrid& g, int free_value = 0)
{
  for (auto& v : g.data) {
    if (v < 0) v = free_value;   // -1 -> 0 (free)
  }
}

LocalizeSubmap::LocalizeSubmap(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
  node_ = config.blackboard->template get<rclcpp::Node::SharedPtr>("node");

  node_->declare_parameter("waypoint_skip_distance", 2.0);
  node_->get_parameter("waypoint_skip_distance", waypoint_skip_distance_);

  waypoints_sub_ = node_->create_subscription<geometry_msgs::msg::PoseArray>(
    "/waypoints_farm", 10,
    [this](geometry_msgs::msg::PoseArray::SharedPtr msg) {
      waypoints_.clear();
      for (auto & p : msg->poses) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header = msg->header;  
        ps.pose   = p;
        waypoints_.push_back(ps);
      }
    });

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  std::string pkg_share = ament_index_cpp::get_package_share_directory("go2_control_cpp");
  node_->get_parameter("maps_dir", maps_dir_param_);

  if (!maps_dir_param_.empty() && maps_dir_param_[0] != '/') {
    maps_dir_param_ = pkg_share + "/" + maps_dir_param_;
  }

  auto yaml_file = maps_dir_param_ + "/map2d.yaml";
  static_map_ = loadMapFromYaml(yaml_file);

  static_map_.header.frame_id = "map2dglobal";
  global_map_pub_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "/map2dglobal", rclcpp::QoS(1).transient_local().reliable());

  submap_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map2d", rclcpp::QoS(1).transient_local().reliable(),
    std::bind(&LocalizeSubmap::submapCallback, this, std::placeholders::_1)
  );

  global_corners_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/objects_global", rclcpp::QoS{10}.transient_local().reliable());
  submap_corners_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/objects_submap", rclcpp::QoS{10}.transient_local().reliable());

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
  last_map2d_tf_.transform = tf2::toMsg(tf2::Transform::getIdentity());

  last_map2d_tf_.header.stamp    = node_->now();
  last_map2d_tf_.header.frame_id = "map2d";       
  last_map2d_tf_.child_frame_id  = "map2dglobal"; 

  tf_broadcaster_->sendTransform(last_map2d_tf_);

  if (!exploration_mode) {
    static_map_.header.stamp = node_->now();
    global_map_pub_->publish(static_map_);
    RCLCPP_INFO(node_->get_logger(), 
      "[LocalizeSubmap] Initialized TF (Identity) and published static global map.");
  }
  
  publish_tf_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(100),
      [this]() {
          std::lock_guard<std::mutex> lock(tf_mutex_);
          if (!last_map2d_tf_.header.frame_id.empty() &&
              !last_map2d_tf_.child_frame_id.empty())
          {
              auto tf_to_pub = last_map2d_tf_;
              tf_to_pub.header.stamp = node_->now();
              tf_broadcaster_->sendTransform(tf_to_pub);
          }
      }
  );
}

BT::PortsList LocalizeSubmap::providedPorts()
{
  return { 
    BT::OutputPort<geometry_msgs::msg::TransformStamped>("map_to_map2d"),
    BT::InputPort<std::vector<Panel>>("global_panels")
  };
}

bool LocalizeSubmap::updateCurrentPoseFromTF()
{
  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_->lookupTransform(
      "odom",          
      "livox_frame",   
      tf2::TimePointZero  
    );
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(node_->get_logger(), "[LocalizeSubmap] TF lookup failed: %s", ex.what());
    return false;
  }

  current_pose_.header.stamp    = rclcpp::Time(tf.header.stamp);
  current_pose_.header.frame_id = "odom";
  current_pose_.pose.position.x = tf.transform.translation.x;
  current_pose_.pose.position.y = tf.transform.translation.y;
  current_pose_.pose.position.z = tf.transform.translation.z;
  current_pose_.pose.orientation = tf.transform.rotation;
  return true;
}

BT::NodeStatus LocalizeSubmap::tick()
{   
  if (exploration_mode) return BT::NodeStatus::SUCCESS;
  if (!updateCurrentPoseFromTF()) return BT::NodeStatus::FAILURE; 
  
  if (!latest_submap_) {
    return BT::NodeStatus::FAILURE;
  }

  nav_msgs::msg::OccupancyGrid local_crop = *latest_submap_; 
  setUnknownToFree(local_crop);

  if (shouldSkipTfBroadcast(waypoint_skip_distance_)) {
    return BT::NodeStatus::SUCCESS;
  }

  geometry_msgs::msg::TransformStamped transform;
  
  if (!alignSubmapToMap(local_crop, transform)) {
    return BT::NodeStatus::SUCCESS; 
  }

  static_map_.header.stamp = node_->now(); 
  global_map_pub_->publish(static_map_);

  setOutput("map_to_map2d", transform);
  return BT::NodeStatus::SUCCESS;
}

std::vector<geometry_msgs::msg::PoseStamped> LocalizeSubmap::convertWaypointsToMapFrame()
{
  std::vector<geometry_msgs::msg::PoseStamped> map_wps;
  map_wps.reserve(waypoints_.size());

  for (auto wp : waypoints_) {
    wp.header.stamp    = rclcpp::Time(0);
    try {
      auto wp_in_map = tf_buffer_->transform(wp, "odom", tf2::durationFromSec(0.1));
      map_wps.push_back(wp_in_map);
    } catch (const tf2::TransformException & ex) {}
  }
  return map_wps;
}

bool LocalizeSubmap::shouldSkipTfBroadcast(double skip_distance)
{
  if (waypoints_.empty()) return false;
  auto map_wps = convertWaypointsToMapFrame();
  
  geometry_msgs::msg::PoseStamped current_in_map;
  try {
    current_in_map = tf_buffer_->transform(current_pose_, "map2d", tf2::durationFromSec(0.1));
  } catch (const tf2::TransformException & ex) {
    return false;
  }

  double min_dist = std::numeric_limits<double>::max();
  for (size_t i = 0; i < map_wps.size(); ++i) {
    double d = nav2_util::geometry_utils::euclidean_distance(current_in_map, map_wps[i]);
    if (d < min_dist) min_dist = d;
  } 
  return min_dist < skip_distance;
}

nav_msgs::msg::OccupancyGrid LocalizeSubmap::loadMapFromYaml(const std::string & yaml_file)
{
  nav_msgs::msg::OccupancyGrid map;
  if (!rcpputils::fs::exists(yaml_file)) {
    exploration_mode = true;
    return map;
  }
  auto status = nav2_map_server::loadMapFromYaml(yaml_file, map);
  if (status != nav2_map_server::LOAD_MAP_SUCCESS) {
    exploration_mode = true;
  }
  return map;
}

void LocalizeSubmap::submapCallback(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
{
  latest_submap_ = *msg;
}

int LocalizeSubmap::calculateOverlapScore(
      const std::vector<cv::Point2f>& submap_points,
      double yaw_offset)
{
    int score = 0;
    double cos_a = std::cos(yaw_offset);
    double sin_a = std::sin(yaw_offset);

    // Global Map Info
    double g_res = static_map_.info.resolution;
    double g_ox  = static_map_.info.origin.position.x;
    double g_oy  = static_map_.info.origin.position.y;
    int g_w      = static_map_.info.width;
    int g_h      = static_map_.info.height;

    for (const auto& p : submap_points) {
        // Rotate the point
        double gx = p.x * cos_a - p.y * sin_a;
        double gy = p.x * sin_a + p.y * cos_a;

        // Convert to Global Grid indices
        int c = static_cast<int>((gx - g_ox) / g_res);
        int r = static_cast<int>((gy - g_oy) / g_res);

        // Check Bounds
        if (c >= 0 && c < g_w && r >= 0 && r < g_h) {
            // Check Occupancy (index = r * width + c)
            if (static_map_.data[r * g_w + c] > 50) { 
                score++;
            }
        }
    }
    return score;
}

bool LocalizeSubmap::alignSubmapToMap(
  const nav_msgs::msg::OccupancyGrid & submap,
  geometry_msgs::msg::TransformStamped & transform)
{
  // 1. Get Robot Position in Submap Frame (map2d) to optimize search zone
  geometry_msgs::msg::PoseStamped robot_in_map2d;
  try {
    robot_in_map2d = tf_buffer_->transform(current_pose_, "map2d", tf2::durationFromSec(0.1));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(node_->get_logger(), "[Align] Cannot transform robot to map2d, skipping optimization.");
    return false;
  }

  double rx = robot_in_map2d.pose.position.x;
  double ry = robot_in_map2d.pose.position.y;
  double search_radius = 10.0; // 10m box

  // 2. Extract Occupied Points Only within 10m Box
  std::vector<cv::Point2f> occupied_points;
  // Reduced reserve size due to stride (approx factor 4 less)
  occupied_points.reserve(1000); 

  double s_res = submap.info.resolution;
  double s_ox  = submap.info.origin.position.x;
  double s_oy  = submap.info.origin.position.y;
  int s_w      = submap.info.width;
  int s_h      = submap.info.height;

  // Calculate Grid Bounds for the 10m box
  int min_x_idx = std::max(0, static_cast<int>((rx - search_radius - s_ox) / s_res));
  int max_x_idx = std::min(s_w, static_cast<int>((rx + search_radius - s_ox) / s_res));
  int min_y_idx = std::max(0, static_cast<int>((ry - search_radius - s_oy) / s_res));
  int max_y_idx = std::min(s_h, static_cast<int>((ry + search_radius - s_oy) / s_res));

  // --- EFFICIENCY IMPROVEMENT ---
  // Stride: Skip every N pixels. 
  // stride=2 means we check 1 out of 4 pixels (2x2 area).
  // stride=3 means 1 out of 9.
  int stride = 2; 

  for (int y = min_y_idx; y < max_y_idx; y += stride) {
      for (int x = min_x_idx; x < max_x_idx; x += stride) {
          int idx = y * s_w + x;
          if (submap.data[idx] > 50) { // Occupied threshold
              double mx = s_ox + (x + 0.5) * s_res;
              double my = s_oy + (y + 0.5) * s_res;
              occupied_points.emplace_back(mx, my);
          }
      }
  }

  if (occupied_points.empty()) {
      return false;
  }

  // 3. Voting Candidates: [Current - Step, Current, Current + Step]
  double step_rad = current_step_deg_ * (M_PI / 180.0);
  
  std::vector<double> candidates = {
      current_yaw_rad_ - step_rad,
      current_yaw_rad_,
      current_yaw_rad_ + step_rad
  };

  int best_score = -1;
  double best_yaw = current_yaw_rad_;

  for (double yaw : candidates) {
      int score = calculateOverlapScore(occupied_points, yaw);
      if (score > best_score) {
          best_score = score;
          best_yaw = yaw;
      }
  }

  // 4. Adaptive Step Logic
  bool stayed_at_center = (std::abs(best_yaw - current_yaw_rad_) < 1e-6);

  if (stayed_at_center) {
      current_step_deg_ *= 0.5;
      if (current_step_deg_ < 0.01) current_step_deg_ = 0.01; 
  } else {
      current_step_deg_ = 1.0;
  }

  current_yaw_rad_ = best_yaw;

  while (current_yaw_rad_ > M_PI)  current_yaw_rad_ -= 2.0 * M_PI;
  while (current_yaw_rad_ < -M_PI) current_yaw_rad_ += 2.0 * M_PI;

  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
      "[Align] Score: %d | Yaw: %.3f deg | Step: %.3f deg", 
      best_score, current_yaw_rad_ * 180.0 / M_PI, current_step_deg_);

  // 5. Construct Transform
  transform.header.stamp    = node_->now();
  transform.header.frame_id = "map2d";
  transform.child_frame_id  = "map2dglobal";
  transform.transform.translation.x = 0.0;
  transform.transform.translation.y = 0.0;
  transform.transform.translation.z = 0.0;
  
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, current_yaw_rad_);
  transform.transform.rotation = tf2::toMsg(q);

  {
      std::lock_guard<std::mutex> lock(tf_mutex_);
      last_map2d_tf_ = transform;
  }
  
  return true;
}

}  // namespace go2_control_cpp

BT_REGISTER_NODES(factory)
{ 
  factory.registerBuilder<go2_control_cpp::LocalizeSubmap>(
    "LocalizeSubmap",
    [](auto & name, auto & config) {
      return std::make_unique<go2_control_cpp::LocalizeSubmap>(name, config);
    });
}