#include "go2_control_cpp/camera_controller.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <sys/stat.h> 
#include <cstdlib>
#include <iomanip>
#include <sstream>

namespace go2_control_cpp
{

CameraController::CameraController(const std::string& name, const BT::NodeConfiguration& config)
  : BT::StatefulActionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  
  cam_pose_pub_ = node_->create_publisher<geometry_msgs::msg::Pose>("/camera_pose", 10);
  photo_req_pub_ = node_->create_publisher<std_msgs::msg::String>("/photo_request_str_fake", 10); // Changed to String for decimal IDs
  video_req_pub_ = node_->create_publisher<std_msgs::msg::String>("/record_video_str", 10); // Changed to String for decimal IDs

  map_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map2d", 
    rclcpp::QoS(10), // <--- CHANGE THIS (Was rclcpp::QoS(1).transient_local())
    std::bind(&CameraController::mapCallback, this, std::placeholders::_1));
  
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);

  current_storage_id_ = "0";
}

void CameraController::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(map_mutex_);
  last_map_ = msg;
}

void CameraController::waypointsCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  last_waypoints_ = msg;
  new_waypoints_available_ = true;
}

BT::PortsList CameraController::providedPorts()
{
  return { BT::InputPort<CurrentPanel>("current_panel") };
}

// Helper to determine the folder name (e.g., "1", "1.1", "1.2")
std::string CameraController::GetNextAvailableFolder(int row_id)
{
    std::string share_dir;
    try {
        share_dir = ament_index_cpp::get_package_share_directory("go2_control_cpp");
    } catch (...) { return std::to_string(row_id); }

    std::string base_path = share_dir + "/photos/" + std::to_string(row_id);
    struct stat info;

    // If base folder doesn't exist, use it
    if (stat(base_path.c_str(), &info) != 0) {
        return std::to_string(row_id);
    }

    // Try suffixes .1 through .9
    for (int i = 1; i <= 9; ++i) {
        std::string suffix_path = base_path + "." + std::to_string(i);
        if (stat(suffix_path.c_str(), &info) != 0) {
            return std::to_string(row_id) + "." + std::to_string(i);
        }
    }

    // Fallback if all 1.1-1.9 are full (just use a timestamp or overwrite 1.9)
    return std::to_string(row_id) + ".9";
}

// ---------------------------------------------------------
// LIFECYCLE METHODS
// ---------------------------------------------------------

BT::NodeStatus CameraController::onStart()
{
  CurrentPanel panel_data;
  if (!getInput("current_panel", panel_data)) {
    RCLCPP_ERROR(node_->get_logger(), "[CamCtrl] Missing current_panel input");
    return BT::NodeStatus::FAILURE; 
  }

  // 1. Check Blackboard first. The DataServer is usually more robust as it runs continuously.
  if (panel_data.scan_direction == "left" || panel_data.scan_direction == "right") {
      active_scan_dir_ = panel_data.scan_direction;
      // We still might want the local lateral distance for safety, 
      // but we respect the DIRECTION from the DataServer.
      double unused_dist; 
      DetermineScanDirection(unused_dist); 
  }
  else {
      // 2. Fallback: Local override if DataServer hasn't decided yet
      double local_lat_dist = 1.0;
      std::string local_scan_dir = DetermineScanDirection(local_lat_dist);
      active_scan_dir_ = local_scan_dir;
      RCLCPP_WARN(node_->get_logger(), "[CamCtrl] Blackboard direction empty, defaulting to local: %s", local_scan_dir.c_str());
  }

  // START RECORDING
  if (!is_recording_) {
      // Logic: Allow re-triggering the same row number
      // We no longer block if panel_data.row_number == last_recorded_row_id_
      
      current_storage_id_ = GetNextAvailableFolder(panel_data.row_number);
      StartRosBag(current_storage_id_, panel_data.lateral_distance);

      std_msgs::msg::String video_msg;
      video_msg.data = current_storage_id_;
      video_req_pub_->publish(video_msg);
      
      is_recording_ = true;
      video_dist_accumulator_ = 0.0;
      current_video_row_ = panel_data.row_number; // The integer row
      last_recorded_row_id_ = panel_data.row_number;
      
      current_row_id_ = panel_data.row_number;
      dist_accumulator_ = 0.0;
      first_tick_ = true; 
      
      RCLCPP_INFO(node_->get_logger(), "[CamCtrl] Video STARTED Folder %s. (Exp: %.2fm)", 
        current_storage_id_.c_str(), panel_data.length);
  }

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus CameraController::onRunning()
{
 
  CurrentPanel panel_data;
  if (!getInput("current_panel", panel_data)) {
    return BT::NodeStatus::FAILURE; 
  }

  // Continually update direction from DataServer updates
  if (panel_data.scan_direction == "left" || panel_data.scan_direction == "right") {
      // Only update if it actually changed to prevent jitter (though string compare is fast enough)
      if (active_scan_dir_ != panel_data.scan_direction) {
          RCLCPP_INFO(node_->get_logger(), "[CamCtrl] Switching direction to %s based on DataServer update", panel_data.scan_direction.c_str());
          active_scan_dir_ = panel_data.scan_direction;
      }
  }

  double table_height    = panel_data.height;
  double bb_lateral_dist = panel_data.lateral_distance;
  std::string scan_dir_str = active_scan_dir_;
  int row_number         = panel_data.row_number;
  double row_yaw_global  = panel_data.row_yaw;
  double panel_length    = panel_data.length;

  if (panel_length < 0.1) panel_length = 100.0;

  if (table_height < 0.5 || table_height > 2.0) table_height = last_valid_height_; 
  else last_valid_height_ = table_height;

  bool scan_right = (scan_dir_str == "right");

  // 2. Logic Reset on Row Change
  if (row_number != current_row_id_) {
      RCLCPP_INFO(node_->get_logger(), "[CamCtrl] Row change (%d -> %d).", current_row_id_, row_number);

      if (is_recording_) StopRecording();

      current_storage_id_ = GetNextAvailableFolder(row_number);
      StartRosBag(current_storage_id_, bb_lateral_dist);

      std_msgs::msg::String video_msg;
      video_msg.data = current_storage_id_;
      video_req_pub_->publish(video_msg);

      is_recording_ = true;
      video_dist_accumulator_ = 0.0;
      current_video_row_ = row_number;
      last_recorded_row_id_ = row_number;
      dist_accumulator_ = 0.0;
      current_row_id_ = row_number;
      first_tick_ = true; 
  }

  // 3. TF Lookup
  geometry_msgs::msg::TransformStamped current_tf;
  try {
    current_tf = tf_buffer_->lookupTransform("odom", "livox_frame", tf2::TimePointZero);
  } catch (tf2::TransformException &ex) {
    return BT::NodeStatus::RUNNING; 
  }

  if (first_tick_ || std::abs(bb_lateral_dist - cached_bb_dist_) > 0.001) {
      anchor_pose_ = current_tf;
      bb_lat_dist_anchor_ = bb_lateral_dist;
      cached_bb_dist_ = bb_lateral_dist;
      last_robot_pose_ = current_tf; 
      first_tick_ = false;
  }

  // 5. Drift Comp & Point Camera
  double dx = current_tf.transform.translation.x - anchor_pose_.transform.translation.x;
  double dy = current_tf.transform.translation.y - anchor_pose_.transform.translation.y;
  
  tf2::Quaternion q_anchor;
  tf2::fromMsg(anchor_pose_.transform.rotation, q_anchor);
  double r, p, yaw_anchor;
  tf2::Matrix3x3(q_anchor).getRPY(r, p, yaw_anchor);

  double global_right_x = std::sin(yaw_anchor);
  double global_right_y = -std::cos(yaw_anchor);
  double lateral_drift  = dx * global_right_x + dy * global_right_y;

  double active_lateral_dist = bb_lat_dist_anchor_;
  if (scan_right) active_lateral_dist -= lateral_drift;
  else            active_lateral_dist += lateral_drift;

  double robot_yaw_map = 0.0;
  try {
      auto map_tf = tf_buffer_->lookupTransform("map2d", "livox_frame", tf2::TimePointZero);
      tf2::Quaternion q_map;
      tf2::fromMsg(map_tf.transform.rotation, q_map);
      double r_map, p_map, y_map;
      tf2::Matrix3x3(q_map).getRPY(r_map, p_map, y_map);
      robot_yaw_map = y_map;
  } catch (...) { robot_yaw_map = 0.0; }

  auto normalize = [](double a) {
      while (a > M_PI) a -= 2.0 * M_PI;
      while (a < -M_PI) a += 2.0 * M_PI;
      return a;
  };

  double rough_look = robot_yaw_map + (scan_right ? -M_PI_2 : M_PI_2);
  double perp1 = normalize(row_yaw_global + M_PI_2);
  double perp2 = normalize(row_yaw_global - M_PI_2);
  double target_yaw_global = (std::abs(normalize(perp1 - rough_look)) < std::abs(normalize(perp2 - rough_look))) ? perp1 : perp2;
  double target_yaw_local = normalize(target_yaw_global - robot_yaw_map);

  PointCamera(active_lateral_dist, last_valid_height_, target_yaw_local);

  // 7. Distance Calculation
  double step_x = current_tf.transform.translation.x - last_robot_pose_.transform.translation.x;
  double step_y = current_tf.transform.translation.y - last_robot_pose_.transform.translation.y;
  double dist_step = std::sqrt(step_x*step_x + step_y*step_y);

  dist_accumulator_ += dist_step;

  if (is_recording_) {
      video_dist_accumulator_ += dist_step;
      if (video_dist_accumulator_ >= panel_length) {
          StopRecording();
      }
  }

  // Trigger Photos - Pass the string ID so photos go to the correct folder (e.g. 1.1)
  double height_rel = last_valid_height_ - CAMERA_Z_WORLD; 
  double dist_3d    = std::sqrt(active_lateral_dist*active_lateral_dist + height_rel*height_rel);
  CheckPhotoTrigger(dist_3d, current_storage_id_);

  last_robot_pose_ = current_tf;
  return BT::NodeStatus::RUNNING;
}

void CameraController::onHalted()
{
  if (is_recording_) StopRecording();
  first_tick_ = true;
  cached_bb_dist_ = -1.0; 
}

// ---------------------------------------------------------
// HELPER METHODS
// ---------------------------------------------------------

void CameraController::StartRosBag(const std::string& storage_id, double initial_dist)
{
    std::string share_dir;
    try {
        share_dir = ament_index_cpp::get_package_share_directory("go2_control_cpp");
    } catch (...) { return; }

    std::string pano_dir = share_dir + "/photos" + "/" + storage_id;
    mkdir(pano_dir.c_str(), 0777); 

    std::string bag_name = "panel_bag_" + storage_id;
    std::string bag_path = pano_dir + "/" + bag_name;

    std::string clean_cmd = "rm -rf " + bag_path;
    int r = std::system(clean_cmd.c_str()); (void)r;

    std::string cmd = "ros2 bag record -s sqlite3 -o " + bag_path + 
                      " /tf /tf_static /odom /livox_frame /camera_pose &";
    
    int result = std::system(cmd.c_str());
    (void)result;

    recording_start_time_ = node_->get_clock()->now();
    recorded_initial_dist_ = initial_dist;
}

void CameraController::StopRecording()
{
    if (!is_recording_) return;

    auto end_time = node_->get_clock()->now();

    std_msgs::msg::String msg;
    msg.data = current_storage_id_;
    video_req_pub_->publish(msg);

    int ret = std::system("pkill -f 'ros2 bag record'"); (void)ret;

    std::string share_dir = ament_index_cpp::get_package_share_directory("go2_control_cpp");
    std::string json_path = share_dir + "/photos/" + current_storage_id_ + "/pano" + current_storage_id_ + ".json";
    
    std::ofstream json_file(json_path);
    if (json_file.is_open()) {
        json_file << "{\n";
        json_file << "  \"row_id\": " << current_video_row_ << ",\n";
        json_file << "  \"storage_id\": \"" << current_storage_id_ << "\",\n";
        json_file << "  \"start_timestamp_nanos\": " << recording_start_time_.nanoseconds() << ",\n";
        json_file << "  \"end_timestamp_nanos\": " << end_time.nanoseconds() << ",\n";
        json_file << "  \"initial_lateral_distance\": " << recorded_initial_dist_ << "\n";
        json_file << "}\n";
        json_file.close();
    }

    RCLCPP_INFO(node_->get_logger(), "[CamCtrl] Stopped Recording Folder %s", current_storage_id_.c_str());

    is_recording_ = false;
    video_dist_accumulator_ = 0.0;
}

void CameraController::PointCamera(double lat_dist, double height_m, double target_yaw)
{
  double height_rel = height_m - CAMERA_Z_WORLD; 
  double pitch_angle = -std::atan2(height_rel, std::abs(lat_dist));

  tf2::Quaternion q;
  q.setRPY(0.0, pitch_angle, target_yaw);

  geometry_msgs::msg::Pose p;
  p.orientation = tf2::toMsg(q);
  cam_pose_pub_->publish(p);

  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = node_->get_clock()->now();
  t.header.frame_id = "livox_frame";
  t.child_frame_id = "camera";
  t.transform.translation.z = 0.2; 
  t.transform.rotation = tf2::toMsg(q);
  tf_broadcaster_->sendTransform(t);
}

void CameraController::CheckPhotoTrigger(double target_dist_3d, const std::string& storage_id)
{
  double coverage_width_m = target_dist_3d * (HORIZ_APERTURE_MM / FOCAL_LENGTH_MM);
  double travel_threshold = coverage_width_m * 0.7; 

  if (dist_accumulator_ >= travel_threshold) {
      std_msgs::msg::String msg;
      msg.data = storage_id;
      photo_req_pub_->publish(msg);
      dist_accumulator_ -= travel_threshold;
  }
}

std::string CameraController::DetermineScanDirection(double& out_lateral_dist)
{
    // Default values if check fails
    std::string direction = "left"; 
    out_lateral_dist = 1.0; 

    // 1. Check for Map
    nav_msgs::msg::OccupancyGrid::SharedPtr current_map;
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        if (!last_map_) {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, 
                "[CamCtrl] No Map received yet. Cannot raytrace.");
            return direction;
        }
        current_map = last_map_; // Shared pointer copy
    }

    // 2. Get Robot Pose (Map Frame)
    geometry_msgs::msg::TransformStamped tf_map;
    try {
        tf_map = tf_buffer_->lookupTransform(current_map->header.frame_id, "livox_frame", tf2::TimePointZero);
    } catch (tf2::TransformException &ex) {
        return direction;
    }

    double rx = tf_map.transform.translation.x;
    double ry = tf_map.transform.translation.y;
    
    tf2::Quaternion q;
    tf2::fromMsg(tf_map.transform.rotation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    // 3. Raytrace Setup
    float res = current_map->info.resolution;
    float ox = current_map->info.origin.position.x;
    float oy = current_map->info.origin.position.y;
    int w = current_map->info.width;
    int h = current_map->info.height;

    auto isOccupied = [&](double x, double y) {
        int ix = static_cast<int>((x - ox) / res);
        int iy = static_cast<int>((y - oy) / res);
        if (ix < 0 || ix >= w || iy < 0 || iy >= h) return false; // Out of bounds is "free" for ray
        int idx = iy * w + ix;
        return current_map->data[idx] > 50; 
    };

    auto castRay = [&](double angle_offset) -> double {
        double theta = yaw + angle_offset;
        double dx = std::cos(theta) * res;
        double dy = std::sin(theta) * res;
        double dist = 0.0;
        double max_dist = 5.0; // Max check 5 meters

        double curr_x = rx;
        double curr_y = ry;

        while (dist < max_dist) {
            if (isOccupied(curr_x, curr_y)) return dist;
            curr_x += dx;
            curr_y += dy;
            dist += res;
        }
        return max_dist;
    };

    // 4. Trace Left (+90) and Right (-90)
    double dist_left = castRay(M_PI_2);
    double dist_right = castRay(-M_PI_2);

    // 5. Determine Result
    if (dist_left < dist_right) {
        direction = "left";
        out_lateral_dist = dist_left;
    } else {
        direction = "right";
        out_lateral_dist = dist_right;
    }
    
    // Add offset for center of structure (approx 0.15m into the foliage)
    out_lateral_dist += 0.15; 

    return direction;
}

} // namespace go2_control_cpp

BT_REGISTER_NODES(factory)
{
  factory.registerBuilder<go2_control_cpp::CameraController>(
    "CameraController",
    [](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<go2_control_cpp::CameraController>(name, config);
    }
  );
}