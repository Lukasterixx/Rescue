#include <memory>
#include <cmath>
#include <algorithm>
#include <vector>
#include <mutex>
#include <cstring>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "map_msgs/msg/occupancy_grid_update.hpp" // Required for updates
#include "geometry_msgs/msg/point_stamped.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/static_transform_broadcaster.h"

using std::placeholders::_1;

class FlattenNode : public rclcpp::Node
{
public:
  FlattenNode()
  : Node("flatten_pointcloud_to_map2d_node")
  {
    // --- Parameters ---
    // Reduced default size to 100m to save CPU/RAM (1000x1000 grid @ 0.1m)
    this->declare_parameter<double>("resolution", 0.1); 
    this->declare_parameter<double>("max_width_m", 200.0); 
    this->declare_parameter<double>("max_height_m", 200.0);
    this->declare_parameter<double>("voxel_res_multiplier", 2.0); // Increased for efficiency

    this->declare_parameter<std::string>("cloud_topic", "/glim_rosnode/points");
    this->declare_parameter<std::string>("map2d_topic", "/map2d");
    this->declare_parameter<std::string>("global_map_topic", "/map2dglobal"); 
    this->declare_parameter<std::string>("target_frame", "odom");
    this->declare_parameter<std::string>("base_frame", "livox_frame");

    this->declare_parameter<int>("queue_size", 10);
    this->declare_parameter<double>("update_radius_m", 10.0);
    this->declare_parameter<double>("tick_rate_hz", 3.0); 
    this->declare_parameter<int>("max_occ", 100);
    // How often to force a full map publish for custom nodes (every N ticks)
    // 3.0Hz tick / 6 = 0.5Hz full update
    this->declare_parameter<int>("full_map_publish_every_n_ticks", 3);

    this->declare_parameter<double>("min_height_rel", 0.0);  
    this->declare_parameter<double>("max_height_rel", 2.5);
    
    // --- NEW PARAMETERS ---
    this->declare_parameter<double>("min_height_close", 0.05); // Default closer to ground
    this->declare_parameter<double>("close_distance", 2.0);    // Radius for close check

    this->get_parameter("resolution", resolution_);
    this->get_parameter("max_width_m", max_width_m_);
    this->get_parameter("max_height_m", max_height_m_);
    this->get_parameter("voxel_res_multiplier", voxel_res_multiplier_);
    this->get_parameter("min_height_rel", min_height_rel_);
    this->get_parameter("max_height_rel", max_height_rel_);
    this->get_parameter("min_height_close", min_height_close_); // Retrieve new param
    this->get_parameter("close_distance", close_distance_);     // Retrieve new param
    this->get_parameter("cloud_topic", cloud_topic_);
    this->get_parameter("map2d_topic", map2d_topic_);
    this->get_parameter("global_map_topic", global_map_topic_);
    this->get_parameter("target_frame", target_frame_);
    this->get_parameter("base_frame", base_frame_);
    this->get_parameter("queue_size", queue_size_);
    this->get_parameter("update_radius_m", update_radius_m_);
    this->get_parameter("tick_rate_hz", tick_rate_hz_);
    this->get_parameter("max_occ", max_occ_);
    this->get_parameter("full_map_publish_every_n_ticks", full_map_skip_);

    if (max_occ_ < 1) max_occ_ = 1;

    // --- Static Transforms ---
    static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    std::vector<geometry_msgs::msg::TransformStamped> static_transforms;
    rclcpp::Time now = this->now();

    // 2. odom -> map2d
    geometry_msgs::msg::TransformStamped tf_odom_map2d;
    tf_odom_map2d.header.stamp = now;
    tf_odom_map2d.header.frame_id = target_frame_; 
    tf_odom_map2d.child_frame_id = "map2d";
    tf_odom_map2d.transform.rotation.w = 1.0;
    static_transforms.push_back(tf_odom_map2d);

    // 3. livox_frame -> base_link
    geometry_msgs::msg::TransformStamped tf_sensor_base;
    tf_sensor_base.header.stamp = now;
    tf_sensor_base.header.frame_id = base_frame_; 
    tf_sensor_base.child_frame_id = "base_link";  
    tf_sensor_base.transform.rotation.w = 1.0;
    static_transforms.push_back(tf_sensor_base);

    static_broadcaster_->sendTransform(static_transforms);

    // --- Compute Fixed Map Size ---
    global_w_ = static_cast<size_t>(std::ceil(max_width_m_ / resolution_));
    global_h_ = static_cast<size_t>(std::ceil(max_height_m_ / resolution_));
    origin_x_ = -0.5 * (double)global_w_ * resolution_;
    origin_y_ = -0.5 * (double)global_h_ * resolution_;

    RCLCPP_INFO(this->get_logger(), "Initialized Map: %zux%zu cells (%.2fm x %.2fm)", 
                global_w_, global_h_, max_width_m_, max_height_m_);

    // --- Pre-allocate Memory ---
    occupancy_counts_.assign(global_w_ * global_h_, -1);
    visit_state_.assign(global_w_ * global_h_, 0);
    last_hit_tick_.assign(global_w_ * global_h_, 0);
    active_indices_.reserve(20000); 

    // --- TF & Subs ---
    tf_buffer_  = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_= std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, queue_size_,
      std::bind(&FlattenNode::cloudCallback, this, _1));

    if (!global_map_topic_.empty()) {
      global_map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        global_map_topic_, rclcpp::QoS(1).transient_local(),
        std::bind(&FlattenNode::globalMapCallback, this, _1));
    }

    // --- Publishers ---
    // 1. Full Map (Throttled, Latched)
    publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
      map2d_topic_, rclcpp::QoS(1).transient_local().reliable());

    // 2. Incremental Updates (High Freq)
    std::string updates_topic = map2d_topic_ + "_updates";
    update_pub_ = this->create_publisher<map_msgs::msg::OccupancyGridUpdate>(
      updates_topic, rclcpp::QoS(10).reliable());

    // Initialize the shared message structure once
    map_msg_.header.frame_id = "map2d"; 
    map_msg_.info.resolution = resolution_;
    map_msg_.info.width = global_w_;
    map_msg_.info.height = global_h_;
    map_msg_.info.origin.position.x = origin_x_;
    map_msg_.info.origin.position.y = origin_y_;
    map_msg_.info.origin.orientation.w = 1.0;
    map_msg_.data.resize(global_w_ * global_h_, -1);

    // Initial publish
    publisher_->publish(map_msg_);

    auto period = std::chrono::duration<double>(1.0 / tick_rate_hz_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&FlattenNode::update, this));
  }

private:
  void cloudCallback(sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    cloud_msg_ = msg;
    have_cloud_ = true;
  }

  void globalMapCallback(nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    global_map_msg_ = msg;
    have_global_map_ = true;
  }

  bool isGlobalMapOccupied(double gx, double gy) 
  {
    if (!have_global_map_ || !global_map_msg_) return false;

    // Assuming global map static properties for simplicity inside loop
    double map_origin_x = global_map_msg_->info.origin.position.x;
    double map_origin_y = global_map_msg_->info.origin.position.y;
    double map_res = global_map_msg_->info.resolution;
    int width = global_map_msg_->info.width;
    int height = global_map_msg_->info.height;

    int idx_x = static_cast<int>((gx - map_origin_x) / map_res);
    int idx_y = static_cast<int>((gy - map_origin_y) / map_res);

    if (idx_x < 0 || idx_x >= width || idx_y < 0 || idx_y >= height) {
      return false; 
    }

    size_t idx = static_cast<size_t>(idx_y) * width + static_cast<size_t>(idx_x);
    int8_t val = global_map_msg_->data[idx];
    return (val > 50);
  }

  void update()
  {
    tick_count_++;

    sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg_in;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (!have_cloud_) return;
      cloud_msg_in = cloud_msg_; 
      have_cloud_ = false; 
    }
    if (!cloud_msg_in) return;

    // --- 1. TF Lookup ---
    geometry_msgs::msg::TransformStamped tf_base;
    geometry_msgs::msg::TransformStamped tf_cloud;
    try {
      tf_base = tf_buffer_->lookupTransform(
        target_frame_, base_frame_, tf2::TimePointZero);
      
      tf_cloud = tf_buffer_->lookupTransform(
          target_frame_,
          cloud_msg_in->header.frame_id,
          cloud_msg_in->header.stamp, 
          tf2::durationFromSec(0.1));
    }
    catch (const tf2::TransformException &ex) {
      return;
    }

    const double cx = tf_base.transform.translation.x;
    const double cy = tf_base.transform.translation.y;
    const double cz = tf_base.transform.translation.z;

    // Global Map TF
    bool global_transform_valid = false;
    geometry_msgs::msg::TransformStamped tf_local_to_global;
    if (!global_map_topic_.empty() && have_global_map_) {
        try {
            std::lock_guard<std::mutex> lock(data_mutex_);
            if(global_map_msg_) {
                tf_local_to_global = tf_buffer_->lookupTransform(
                global_map_msg_->header.frame_id, 
                target_frame_, 
                tf2::TimePointZero);
                global_transform_valid = true;
            }
        } catch (const tf2::TransformException &ex) {}
    }

    // --- 2. FAST Voxel Filter (No PCL) ---
    sensor_msgs::msg::PointCloud2 cloud_tf;
    tf2::doTransform(*cloud_msg_in, cloud_tf, tf_cloud);

    // Precompute height thresholds (absolute Z)
    const double min_z_far = cz + min_height_rel_;
    const double min_z_close = cz + min_height_close_;
    const double max_allowed_z = cz + max_height_rel_;
    
    // Radii squared
    const double r2_update = update_radius_m_ * update_radius_m_;
    const double r2_close = close_distance_ * close_distance_;
    
    active_indices_.clear();

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_tf, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_tf, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_tf, "z");

    int coarse_mult = static_cast<int>(voxel_res_multiplier_);
    if (coarse_mult < 1) coarse_mult = 1;

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
    {
        float px = *iter_x;
        float py = *iter_y;
        float pz = *iter_z;

        if (!std::isfinite(px)) continue;

        // Calculate distance first to determine which height threshold to use
        double dx = px - cx;
        double dy = py - cy;
        double dist_sq = dx*dx + dy*dy;

        // Apply distance limit immediately
        if (dist_sq > r2_update) continue;

        // Determine minimum Z based on distance
        double current_min_z = (dist_sq < r2_close) ? min_z_close : min_z_far;

        // Height Check
        if (pz < current_min_z || pz > max_allowed_z) continue;

        int ix = static_cast<int>((px - origin_x_) / resolution_);
        int iy = static_cast<int>((py - origin_y_) / resolution_);

        // Snap to coarse grid for deduplication
        int coarse_ix = (ix / coarse_mult) * coarse_mult;
        int coarse_iy = (iy / coarse_mult) * coarse_mult;

        if (coarse_ix >= 0 && coarse_ix < (int)global_w_ && coarse_iy >= 0 && coarse_iy < (int)global_h_) {
             size_t combined_idx = (size_t)coarse_iy * global_w_ + (size_t)coarse_ix;
             active_indices_.push_back(combined_idx);
        }
    }

    std::sort(active_indices_.begin(), active_indices_.end());
    auto last = std::unique(active_indices_.begin(), active_indices_.end());
    active_indices_.erase(last, active_indices_.end());

    // --- 3. Paint ---
    int brush_rad = static_cast<int>(voxel_res_multiplier_ / 2.0);
    if (brush_rad < 0) brush_rad = 0;

    for (size_t center_idx : active_indices_) {
        int base_iy = center_idx / global_w_;
        int base_ix = center_idx % global_w_;

        for (int dy = -brush_rad; dy <= brush_rad; dy++) {
            for (int dx = -brush_rad; dx <= brush_rad; dx++) {
                int nx = base_ix + dx;
                int ny = base_iy + dy;

                if (nx < 0 || nx >= (int)global_w_ || ny < 0 || ny >= (int)global_h_) continue;
                
                size_t idx = (size_t)ny * global_w_ + (size_t)nx;
                
                int c = occupancy_counts_[idx];
                if (c < 0) c = 0;
                c += 300; 
                if (c > max_occ_) c = max_occ_;
                occupancy_counts_[idx] = (int8_t)c;
                last_hit_tick_[idx] = tick_count_;
            }
        }
    }

    // --- 4. Decay (ROI Only) ---
    int ci = static_cast<int>((cx - origin_x_) / resolution_);
    int cj = static_cast<int>((cy - origin_y_) / resolution_);
    int ri = static_cast<int>(update_radius_m_ / resolution_);
    
    // Bounds for Update Message
    int roi_min_x = std::max(0, ci - ri - 2);
    int roi_max_x = std::min((int)global_w_ - 1, ci + ri + 2);
    int roi_min_y = std::max(0, cj - ri - 2);
    int roi_max_y = std::min((int)global_h_ - 1, cj + ri + 2);

    for (int y = roi_min_y; y <= roi_max_y; ++y) {
        for (int x = roi_min_x; x <= roi_max_x; ++x) {
            
            double wx = origin_x_ + (x + 0.5) * resolution_;
            double wy = origin_y_ + (y + 0.5) * resolution_;
            if ((wx - cx)*(wx - cx) + (wy - cy)*(wy - cy) > r2_update) continue;

            size_t idx = (size_t)y * global_w_ + (size_t)x;

            if (visit_state_[idx] < 2) visit_state_[idx]++;
            if (last_hit_tick_[idx] == tick_count_) continue;

            int c = occupancy_counts_[idx];
            if (c <= 0) {
                if (c < 0) occupancy_counts_[idx] = 0;
                continue;
            }

            bool is_static = false;
            if (global_transform_valid) {
                 geometry_msgs::msg::PointStamped pt_in, pt_out;
                 pt_in.point.x = wx; pt_in.point.y = wy; pt_in.point.z = 0;
                 tf2::doTransform(pt_in.point, pt_out.point, tf_local_to_global);
                 if (isGlobalMapOccupied(pt_out.point.x, pt_out.point.y)) is_static = true;
            }

            if (is_static) continue; 

            c -= 0; 
            if (c < 0) c = 0;
            occupancy_counts_[idx] = (int8_t)c;
        }
    }

    // --- 5. Publish Update (Every Tick -> Fast) ---
    map_msgs::msg::OccupancyGridUpdate update_msg;
    update_msg.header.stamp = this->now();
    update_msg.header.frame_id = "map2d";
    update_msg.x = roi_min_x;
    update_msg.y = roi_min_y;
    update_msg.width  = (roi_max_x - roi_min_x) + 1;
    update_msg.height = (roi_max_y - roi_min_y) + 1;
    update_msg.data.resize(update_msg.width * update_msg.height);

    for (uint32_t y = 0; y < update_msg.height; ++y) {
        for (uint32_t x = 0; x < update_msg.width; ++x) {
            size_t g_idx = (size_t)(y + roi_min_y) * global_w_ + (size_t)(x + roi_min_x);
            size_t l_idx = (size_t)y * update_msg.width + (size_t)x;
            int8_t val = occupancy_counts_[g_idx];
            
            if (val < 0) update_msg.data[l_idx] = -1;
            else if (val == 0) update_msg.data[l_idx] = 0;
            else {
                 int v = (int)val * 100 / max_occ_;
                 if(v > 100) v = 100;
                 update_msg.data[l_idx] = (int8_t)v;
            }
        }
    }
    update_pub_->publish(update_msg);

    // --- 6. Publish Full Map (Throttled -> Efficient) ---
    if (tick_count_ % full_map_skip_ == 0) {
        map_msg_.header.stamp = this->now();
        // Memcpy is fine here because logic is simple, but mapping requires loop
        // Reuse buffer to avoid resize
        const size_t n = occupancy_counts_.size();
        const int8_t* src = occupancy_counts_.data();
        int8_t* dst = map_msg_.data.data();

        for (size_t i = 0; i < n; ++i) {
            int8_t val = src[i];
            if (val < 0) dst[i] = -1;
            else if (val == 0) dst[i] = 0;
            else {
                 int v = (int)val * 100 / max_occ_;
                 if(v > 100) v = 100;
                 dst[i] = (int8_t)v;
            }
        }
        publisher_->publish(map_msg_);
    }
  }

  // Params
  double resolution_;
  double max_width_m_;
  double max_height_m_;
  double voxel_res_multiplier_;
  double min_height_rel_;
  double max_height_rel_;
  double min_height_close_; // New Param
  double close_distance_;   // New Param
  
  std::string cloud_topic_;
  std::string map2d_topic_;
  std::string global_map_topic_;
  std::string target_frame_;
  std::string base_frame_;

  int queue_size_;
  double update_radius_m_;
  double tick_rate_hz_;
  int max_occ_;
  int full_map_skip_;

  size_t global_w_{0};
  size_t global_h_{0};
  double origin_x_{0.0};
  double origin_y_{0.0};

  std::vector<int8_t> occupancy_counts_;
  std::vector<uint8_t> visit_state_;
  std::vector<uint32_t> last_hit_tick_;
  std::vector<size_t> active_indices_;
  
  uint32_t tick_count_{0};

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr global_map_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
  rclcpp::Publisher<map_msgs::msg::OccupancyGridUpdate>::SharedPtr update_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;

  std::mutex data_mutex_;
  sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg_;
  nav_msgs::msg::OccupancyGrid::SharedPtr global_map_msg_;
  bool have_cloud_{false};
  bool have_global_map_{false};

  nav_msgs::msg::OccupancyGrid map_msg_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FlattenNode>());
  rclcpp::shutdown();
  return 0;
}