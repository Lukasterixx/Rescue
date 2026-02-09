// Updated data_server.cpp
#include "go2_control_cpp/path_planner.hpp"
#include "go2_control_cpp/data_server.hpp"
#include <behaviortree_cpp_v3/bt_factory.h>
#include "ament_index_cpp/get_package_share_directory.hpp"

// Required for cloud transformation
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp> 
// Required for Pose publishing
#include <geometry_msgs/msg/pose.hpp>

namespace go2_control_cpp
{

DataServer::DataServer(
    const std::string& xml_tag_name, 
    const BT::NodeConfiguration& config)
: BT::SyncActionNode(xml_tag_name, config)
{
  node_ = this->config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  // Declare & get parameters
  node_->declare_parameter<double>("table_height",      0.9);
  node_->declare_parameter<double>("table_tilt",        17.0);
  node_->declare_parameter<double>("unit_height",   2276.0);
  node_->declare_parameter<double>("unit_width",    1134.0);
  node_->declare_parameter<double>("unit_sep",  50.0);

  node_->get_parameter("table_height",      table_height_m_);
  node_->get_parameter("table_tilt",        table_tilt_deg_);
  node_->get_parameter("unit_height",   unit_height_mm_);
  node_->get_parameter("unit_width",    unit_width_mm_);
  node_->get_parameter("unit_sep",      unit_sep_mm_);

  RCLCPP_INFO(node_->get_logger(),
    "DataServer params: table_height=%.2fm, tilt=%.1f°, panel H=%.0fmm, W=%.0fmm, sep=%.0fmm",
    table_height_m_, table_tilt_deg_, unit_height_mm_, unit_width_mm_, unit_sep_mm_);

  // Subscriptions
  map_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map2d", 10,
    std::bind(&DataServer::mapCallback, this, std::placeholders::_1));

  map_cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/glim_rosnode/points", 10,
    std::bind(&DataServer::cloudCallback, this, std::placeholders::_1));

  waypoint_sub_ = node_->create_subscription<geometry_msgs::msg::PoseArray>(
    "/waypoints_farm", 10,
    std::bind(&DataServer::waypointsCallback, this, std::placeholders::_1));

  // Publishers
  marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
    "crop_box_marker", rclcpp::QoS{1}.transient_local());

  units_marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/unit_markers", rclcpp::QoS{1});

  // NEW: Camera Control Publisher
  camera_pub_ = node_->create_publisher<geometry_msgs::msg::Pose>(
    "/camera_pose", 10);

  // Initialize point cloud pointer
  last_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());

  // TF broadcaster
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);

   // broadcast the "units" transform at 10 Hz, using the stored avg_angle_deg_
  tf_timer_ = node_->create_wall_timer(
    std::chrono::milliseconds(100),
    [this]()
    {
      BroadcastUnitsFrame(0.0);
    });

  // create TF buffer & listener
  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

bool DataServer::updatePoseFromTF()
{
  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_->lookupTransform(
      "odom",          // target
      "livox_frame",   // source
      tf2::TimePointZero);
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN(node_->get_logger(),
                "[DataServer] TF lookup odom->livox_frame failed: %s", ex.what());
    return false;
  }

  auto pose = std::make_shared<geometry_msgs::msg::PoseStamped>();
  pose->header = tf.header;

  pose->pose.position.x = tf.transform.translation.x;
  pose->pose.position.y = tf.transform.translation.y;
  pose->pose.position.z = tf.transform.translation.z;

  pose->pose.orientation.x = tf.transform.rotation.x;
  pose->pose.orientation.y = tf.transform.rotation.y;
  pose->pose.orientation.z = tf.transform.rotation.z;
  pose->pose.orientation.w = tf.transform.rotation.w;

  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_pose_ = pose;
  }

  return true;
}


void DataServer::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  last_map_ = msg;
}

void DataServer::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  sensor_msgs::msg::PointCloud2 msg_odom;
  try {
    geometry_msgs::msg::TransformStamped tf_stamped = tf_buffer_->lookupTransform(
      "odom", 
      msg->header.frame_id, 
      tf2::TimePointZero,
      std::chrono::milliseconds(50) 
    );
    tf2::doTransform(*msg, msg_odom, tf_stamped);
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN(node_->get_logger(), "[DataServer] Cloud transform failed: %s", ex.what());
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  pcl::fromROSMsg(msg_odom, *last_cloud_);
}

void DataServer::waypointsCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  last_waypoints_ = msg;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////

BT::NodeStatus DataServer::tick()
{ 
  // --- Exploration mode check ---
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const bool no_waypoint_publishers = (node_->count_publishers("/waypoints_farm") == 0);
    const bool no_waypoint_msgs       = (!last_waypoints_ || last_waypoints_->poses.empty());

    exploration_mode_ = (no_waypoint_publishers || no_waypoint_msgs);

    if (exploration_mode_) {
      RCLCPP_WARN(node_->get_logger(),
        "[DataServer] exploration_mode enabled. Returning SUCCESS immediately.");
      return BT::NodeStatus::SUCCESS;
    }
  }
  
  if (!LoadPanels(tables_)) {
    return BT::NodeStatus::FAILURE;
  }

  if (!updatePoseFromTF()) {
    return BT::NodeStatus::FAILURE;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (!last_map_ || !last_pose_ || !last_cloud_ || !last_waypoints_ || last_waypoints_->poses.empty()) {
    RCLCPP_WARN(node_->get_logger(), "Waiting for all inputs (map, cloud, pose, waypoints)...");
    return BT::NodeStatus::FAILURE;
  }

  // Only update and publish if we actually see a panel
  if (UpdatePanelDimensions()) {
    GenerateJSON();
    BroadcastUnitsFrame(0.0);
    PublishUnitMarkers(tables_);
    
    RCLCPP_INFO(node_->get_logger(),
      "Updated scan: height=%.2fm, tilt=%.1f°", table_height_m_, table_tilt_deg_);
  } else {
    // If UpdatePanelDimensions returns false, we do nothing.
    // The previous markers remain in RViz because we haven't sent a DELETE action.
    RCLCPP_DEBUG(node_->get_logger(), "No valid panel scan this tick, retaining previous data.");
  }

  return BT::NodeStatus::SUCCESS;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////



void DataServer::BroadcastUnitsFrame(double yaw_deg)
{
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp    = node_->now();
  t.header.frame_id = "map2d";
  t.child_frame_id  = "units";
  t.transform.translation.x = 0.0;
  t.transform.translation.y = 0.0;
  t.transform.translation.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0,0,-yaw_deg * M_PI/180.0);
  t.transform.rotation = tf2::toMsg(q);
  tf_broadcaster_->sendTransform(t);
}

void DataServer::PublishUnitMarkers(const std::vector<Table>& tables)
{
  visualization_msgs::msg::MarkerArray arr;
  int id = 0;
  const double tilt_rad = table_tilt_deg_ * M_PI / 180.0;

  for (const auto &t : tables)
  {
    // Calculate the center of the table structure in the tilt-axis (Y)
    // This matches where the RANSAC average height was likely calculated
    float table_center_y = t.center.y; 

    for (size_t r = 0; r < t.units.size(); ++r)
    {
      for (const auto &u : t.units[r])
      {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "units";
        m.header.stamp    = node_->now();
        m.ns              = "units";
        m.id              = id++;
        m.type            = m.LINE_STRIP;
        m.action          = m.ADD;
        m.lifetime = rclcpp::Duration::from_seconds(0); // 0 means forever
        m.scale.x         = 0.1;
        m.color.r         = 1.0f;
        m.color.g         = 0.5f; // Added green to make it orange/gold
        m.color.a         = 0.8f;

        // Draw the 4 corners + closing point
        const int order[5] = {0, 1, 3, 2, 0};
        for (int idx : order)
        {
          geometry_msgs::msg::Point p;
          p.x = u.corners[idx].x;
          p.y = u.corners[idx].y;

          // The vertical height (Z) is the base height PLUS the rise/fall 
          // relative to the center of the table.
          double delta_y_from_center = p.y - table_center_y;
          p.z = table_height_m_ + (delta_y_from_center * std::tan(tilt_rad));

          m.points.push_back(p);
        }
        arr.markers.push_back(std::move(m));
      }
    }
  }
  units_marker_pub_->publish(arr);
}

bool DataServer::LoadPanels(std::vector<Table>& tables)
{
    // ... (Existing implementation) ...
    // [Copy your LoadPanels function here - no changes needed]
  std::vector<Panel> panels;
  if (!getInput("panels", panels)) {
    RCLCPP_ERROR(node_->get_logger(), "[DataServer] failed to read 'panels'");
    return false;
  }
  const auto& info  = last_map_->info;
  double resolution  = info.resolution;
  double origin_x    = info.origin.position.x;
  double origin_y    = info.origin.position.y;
  int    height_px   = info.height;

  auto pixelToWorld = [&](const cv::Point2f& pix){
    double wx = pix.x * resolution + origin_x;
    double wy = (height_px - pix.y - 1) * resolution + origin_y;
    return cv::Point2f(float(wx), float(wy));
  };

  tables.clear();
  tables.reserve(panels.size());
  for (auto const &p : panels)
  {
    Table t;
    t.row = p.row; t.direction = p.direction; t.is_first = p.is_first; t.is_half = false;
    for (int i = 0; i < 4; ++i) t.corners[i] = pixelToWorld(p.corners[i]);
    t.center = pixelToWorld(p.center);
    tables.push_back(std::move(t));
  }

  double unit_w_m = unit_width_mm_  / 1000.0;
  double unit_h_m = unit_height_mm_ / 1000.0;
  double sep_m    = unit_sep_mm_    / 1000.0;

  for (auto &t : tables)
  {
    cv::Point2f TL = t.corners[0]; cv::Point2f TR = t.corners[1]; cv::Point2f BL = t.corners[2];
    cv::Point2f edge_h = TR - TL; float len_h = cv::norm(edge_h); edge_h *= (1.0f/len_h);
    cv::Point2f edge_v = BL - TL; float len_v = cv::norm(edge_v); edge_v *= (1.0f/len_v);

    cv::Point2f axis_long, axis_short; float long_m;
    if (len_h >= len_v) { axis_long = edge_h; long_m = len_h; axis_short = edge_v; } 
    else { axis_long = edge_v; long_m = len_v; axis_short = edge_h; }

    t.angle_rad = std::atan2(axis_long.y, axis_long.x);

    int num_cols = int(std::floor((long_m - sep_m) / (unit_w_m + sep_m)));
    num_cols = std::max(1, num_cols);
    const int num_rows = 2;

    t.units.assign(num_rows, std::vector<Unit>(num_cols));
    cv::Point2f origin = TL;

    for (int r = 0; r < num_rows; ++r) {
      for (int c = 0; c < num_cols; ++c) {
        float off_long  = sep_m + (unit_w_m*0.5f) + c*(unit_w_m + sep_m);
        float off_short = sep_m + (unit_h_m*0.5f) + r*(unit_h_m + sep_m);
        cv::Point2f center = origin + axis_long * off_long + axis_short * off_short;
        cv::Point2f half_w = axis_long  * (unit_w_m*0.5f);
        cv::Point2f half_h = axis_short * (unit_h_m*0.5f);
        cv::Point2f uTL = center - half_w - half_h; cv::Point2f uTR = center + half_w - half_h;
        cv::Point2f uBL = center - half_w + half_h; cv::Point2f uBR = center + half_w + half_h;

        Unit U; U.row = r; U.col = c; U.first_col_right = false; U.center = center;
        U.corners = { uTL, uTR, uBL, uBR }; U.direction = t.direction; 
        U.is_first = (t.is_first && r == 0 && c == 0);
        t.units[r][c] = U;
      }
    }
  }
  return true;
}

bool DataServer::GenerateJSON()
{
  // ... (Existing implementation) ...
  // [Copy your GenerateJSON function here]
  std::string maps_dir;
  if (!node_->get_parameter("maps_dir", maps_dir)) return false;

  nlohmann::json j;
  j["tables"] = nlohmann::json::array();

  for (const auto &t : tables_) {
    nlohmann::json jt;
    jt["row"] = t.row; jt["direction"] = t.direction; jt["is_first"] = t.is_first; jt["is_half"] = t.is_half;
    jt["center"] = { {"x", t.center.x}, {"y", t.center.y} };
    jt["corners"] = nlohmann::json::array();
    for (const auto &pt : t.corners) jt["corners"].push_back({ {"x", pt.x}, {"y", pt.y} });
    jt["units"] = nlohmann::json::array();
    for (const auto &row : t.units) {
      nlohmann::json jrow = nlohmann::json::array();
      for (const auto &u : row) {
        nlohmann::json ju;
        ju["row"] = u.row; ju["col"] = u.col; ju["first_col_right"] = u.first_col_right;
        ju["direction"] = u.direction; ju["is_first"] = u.is_first;
        ju["center"] = { {"x", u.center.x}, {"y", u.center.y} };
        ju["corners"] = nlohmann::json::array();
        for (const auto &up : u.corners) ju["corners"].push_back({ {"x", up.x}, {"y", up.y} });
        jrow.push_back(std::move(ju));
      }
      jt["units"].push_back(std::move(jrow));
    }
    j["tables"].push_back(std::move(jt));
  }

  if (!maps_dir.empty() && maps_dir[0] != '/') {
    auto pkg_share = ament_index_cpp::get_package_share_directory("go2_control_cpp");
    maps_dir = pkg_share + "/" + maps_dir;
  }
  const std::string out_path = maps_dir + "/tables.json";
  std::ofstream ofs(out_path);
  if (!ofs) return false;
  ofs << std::setw(2) << j << std::endl;
  return true;
}

bool DataServer::UpdatePanelDimensions()
{
  bool success = false; // true if analysis worked

  // --- A. Transform Robot Pose to Map2D ---
  geometry_msgs::msg::PoseStamped r_in, r2d;
  r_in.header = last_pose_->header;
  r_in.pose   = last_pose_->pose;
  r_in.header.stamp = rclcpp::Time(0); 

  try {
    r2d = tf_buffer_->transform(r_in, "map2d", tf2::durationFromSec(0.1));
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(node_->get_logger(), "[DataServer] Transform robot->map2d failed: %s", ex.what());
    return false;
  }

  double rx_2d = r2d.pose.position.x;
  double ry_2d = r2d.pose.position.y;
  double robot_yaw_2d = tf2::getYaw(r2d.pose.orientation);

  // --- B. Raytrace (Keep logic to detect proximity/walls) ---
  const auto& info  = last_map_->info;
  double resolution = info.resolution;
  double origin_x   = info.origin.position.x;
  double origin_y   = info.origin.position.y;
  int width         = info.width;
  int height        = info.height;

  auto occupiedAt = [&](const Eigen::Vector2f &pt) {
    int mx = int((pt.x() - origin_x) / resolution);
    int my = int((pt.y() - origin_y) / resolution);
    if (mx < 0 || mx >= width || my < 0 || my >= height) return false;
    int idx = my * width + mx;
    return last_map_->data[idx] >= 50;
  };

  const float ray_dist = 10.0f;
  const float ray_step = static_cast<float>(resolution);
  
  auto traceSide = [&](float angle_offset) -> float {
    float angle = robot_yaw_2d + angle_offset;
    Eigen::Vector2f dir(std::cos(angle), std::sin(angle));
    for (float d = 0.1f; d <= ray_dist; d += ray_step) {
      Eigen::Vector2f p(rx_2d + d * dir.x(), ry_2d + d * dir.y());
      if (occupiedAt(p)) return d;
    }
    return ray_dist;
  };

  // 1. Get physical distances from Map
  float dist_left  = traceSide(M_PI_2);
  float dist_right = traceSide(-M_PI_2);

  // 2. Check if ANYTHING is close (Raytrace detection trigger)
  // If both sides are empty space (max distance), we might assume no panel is close.
  // However, we continue to the geometric check to be safe.
  bool map_obstacle_detected = (dist_left < ray_dist || dist_right < ray_dist);

  // FIX: Use the variable to exit early if nothing is close
  if (!map_obstacle_detected) {
      return false;
  }

  // --- C. Geometric Decision (Override Left/Right using Tables) ---
  // We determine direction based on the closest panel center, NOT the raytrace.
  
  const Table* closest_table = nullptr;
  double min_dist_sq = std::numeric_limits<double>::max();
  bool scan_right = false; // Will be set by geometry

  if (!tables_.empty()) {
    for (const auto& t : tables_) {
      double dx = t.center.x - rx_2d;
      double dy = t.center.y - ry_2d;
      double dist_sq = dx*dx + dy*dy;

      if (dist_sq < min_dist_sq) {
        min_dist_sq = dist_sq;
        closest_table = &t;
      }
    }
  }

  if (closest_table) {
    // Calculate determining side:
    // Transform vector-to-table into Robot Local Frame.
    // Local Y > 0 is Left, Local Y < 0 is Right.
    double dx = closest_table->center.x - rx_2d;
    double dy = closest_table->center.y - ry_2d;
    double local_y = -dx * std::sin(robot_yaw_2d) + dy * std::cos(robot_yaw_2d);

    scan_right = (local_y < 0);
  } else {
    // Fallback: If no tables exist (unlikely if logic is correct), use raytrace
    scan_right = (dist_right < dist_left);
  }

  std::string direction_str = scan_right ? "right" : "left";

  // --- D. Point Cloud Crop & Plane Fitting ---
  // Now we set the crop box based on the GEOMETRIC decision (scan_right)
  const float box_dim = 10.0f; 
  float min_x_local = -box_dim/2.0f; float max_x_local = box_dim/2.0f;
  float min_y_local = scan_right ? -box_dim : 0.0f;
  float max_y_local = scan_right ? 0.0f : box_dim;

  tf2::Quaternion q_map;
  tf2::fromMsg(last_pose_->pose.orientation, q_map);
  double robot_yaw_map = tf2::getYaw(q_map);
  
  pcl::CropBox<pcl::PointXYZ> crop;
  crop.setInputCloud(last_cloud_);
  crop.setRotation(Eigen::Vector3f(0.f, 0.f, robot_yaw_map));
  crop.setTranslation(Eigen::Vector3f(last_pose_->pose.position.x, last_pose_->pose.position.y, 0.f));
  crop.setMin(Eigen::Vector4f(min_x_local, min_y_local, last_pose_->pose.position.z, 1.f));
  crop.setMax(Eigen::Vector4f(max_x_local, max_y_local, std::numeric_limits<float>::max(), 1.f));

  pcl::PointCloud<pcl::PointXYZ> cropped;
  crop.filter(cropped);

  if (!cropped.empty()) {
    // Perform RANSAC
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.05f);
    seg.setInputCloud(cropped.makeShared());

    pcl::ModelCoefficients::Ptr coeffs(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    seg.segment(*inliers, *coeffs);

    if (!inliers->indices.empty()) {

      double sum_z = 0.0;
      for (auto idx : inliers->indices) sum_z += cropped.points[idx].z;
      float detected_h = sum_z / inliers->indices.size();

      // Check height tolerance
      if (std::abs(detected_h - table_height_m_) <= 0.5) { 
          
          success = true; 

          Eigen::Vector3f normal(coeffs->values[0], coeffs->values[1], coeffs->values[2]);
          if (normal.z() < 0) normal = -normal;

          Eigen::Vector2f robot_left(-std::sin(robot_yaw_map), std::cos(robot_yaw_map));
          float lateral_proj = normal.x() * robot_left.x() + normal.y() * robot_left.y();
          
          double tilt_val = std::atan2(lateral_proj, normal.z()) * 180.0 / M_PI;

          // Orientation Correction
          // If we are looking Right, but the robot is rotated 180 (facing West),
          // the tilt calculation needs to be inverted relative to the global Y axis.
          if (closest_table && std::abs(robot_yaw_2d) > M_PI_2) {
             tilt_val = -tilt_val;
          }

          table_tilt_deg_ = tilt_val;
          table_height_m_ = detected_h;
      }
      else {
          RCLCPP_DEBUG(node_->get_logger(), 
            "Plane rejected: height %.2fm is too far from expected %.2fm", 
            detected_h, table_height_m_);
      }
    }
  }

  // --- E. Populate and Export CurrentPanel Struct ---
  CurrentPanel current_panel;

  current_panel.height = table_height_m_;
  current_panel.tilt   = table_tilt_deg_;
  current_panel.scan_direction = direction_str;

  // Use the Raytrace distance corresponding to the CHOSEN side
  double target_lateral_dist = scan_right ? dist_right : dist_left;
  double unit_h_m = unit_height_mm_ / 1000.0;
  double sep_m    = unit_sep_mm_    / 1000.0;
  // Assuming 2 rows in the structure
  double table_structure_width = (2 * unit_h_m) + (1 * sep_m);
  
  current_panel.lateral_distance = target_lateral_dist + (table_structure_width / 2.0);

  if (closest_table) {
      current_panel.row_number = closest_table->row;
      current_panel.row_yaw    = closest_table->angle_rad; 
      current_panel.center  = closest_table->center;
      current_panel.corners = closest_table->corners;

      cv::Point2f edge_top = closest_table->corners[1] - closest_table->corners[0];
      current_panel.length = cv::norm(edge_top);

      cv::Point2f edge_side = closest_table->corners[2] - closest_table->corners[0];
      current_panel.width = cv::norm(edge_side);
  } else {
      current_panel.row_number = -1;
      current_panel.row_yaw    = 0.0;
      current_panel.length     = 0.0;
      current_panel.width      = 0.0;
      current_panel.center     = cv::Point2f(0,0);
  }

  setOutput("current_panel", current_panel);

  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
    "Updated Panel: Row %d, H: %.2f, Tilt: %.1f, Dir: %s, LatDist: %.2f", 
    current_panel.row_number, current_panel.height, current_panel.tilt, 
    direction_str.c_str(), current_panel.lateral_distance);

  return success;
}

} // namespace go2_control_cpp

// Register node as plugin
BT_REGISTER_NODES(factory)
{ 
  factory.registerBuilder<go2_control_cpp::DataServer>(
    "DataServer",
    [](auto & name, auto & config) {
      return std::make_unique<go2_control_cpp::DataServer>(name, config);
    });
}