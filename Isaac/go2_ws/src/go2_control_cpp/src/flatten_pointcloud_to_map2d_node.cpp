// File: src/flatten_pointcloud_to_map2d_node.cpp

#include "go2_control_cpp/flatten_pointcloud_to_map2d_node.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <filesystem>
#include <behaviortree_cpp_v3/tree_node.h>

namespace go2_control_cpp
{

void FlattenPointCloudToMap2D::loadGrid()
{
  grid_.assign(global_w_ * global_h_, -1);
}

void FlattenPointCloudToMap2D::publishFullGrid()
{
  nav_msgs::msg::OccupancyGrid og;
  og.header.stamp    = current_pose_stamp_;
  og.header.frame_id = "map";
  og.info.resolution = resolution_;
  og.info.width      = global_w_;
  og.info.height     = global_h_;
  og.info.origin.position.x    = origin_x_;
  og.info.origin.position.y    = origin_y_;
  og.info.origin.orientation.w = 1.0;
  og.data = grid_;  
  publisher_->publish(og);
}

void FlattenPointCloudToMap2D::poseCallback(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  current_pose_ = msg->pose;
  current_pose_stamp_ = msg->header.stamp;
  have_pose_    = true;
}

void FlattenPointCloudToMap2D::cloudCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  cloud_msg_  = msg;
  have_cloud_ = true;
}

FlattenPointCloudToMap2D::FlattenPointCloudToMap2D(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & config)
  : BT::ActionNodeBase(xml_tag_name, config)
{
  node_ = this->config().blackboard
    ->get<rclcpp::Node::SharedPtr>("node");

  getInput("resolution",              resolution_);
  getInput("max_width_m",             max_width_m_);
  getInput("max_height_m",            max_height_m_);
  getInput("ground_height_threshold", ground_height_thresh_);
  getInput("cloud_topic",             cloud_topic_);
  getInput("pose_topic",              pose_topic_);
  getInput("map2d_topic",             map2d_topic_);
  getInput("queue_size",              queue_size_);
  getInput("update_radius_m",         update_radius_m_);
  getInput("maps_dir",                maps_dir_);

  global_w_ = static_cast<size_t>(max_width_m_ / resolution_) + 1;
  global_h_ = static_cast<size_t>(max_height_m_ / resolution_) + 1;
  origin_x_ = -0.5 * global_w_ * resolution_;
  origin_y_ = -0.5 * global_h_ * resolution_;
  std::filesystem::create_directories(maps_dir_);
  {
    std::string path = maps_dir_ + "/map2d.bin";
    std::ofstream out(path, std::ios::binary|std::ios::trunc);
    uint32_t w = global_w_, h = global_h_;
    out.write(reinterpret_cast<char*>(&w), sizeof(w));
    out.write(reinterpret_cast<char*>(&h), sizeof(h));
    out.write(reinterpret_cast<char*>(&resolution_), sizeof(resolution_));
    out.write(reinterpret_cast<char*>(&origin_x_), sizeof(origin_x_));
    out.write(reinterpret_cast<char*>(&origin_y_), sizeof(origin_y_));
    std::vector<uint8_t> blank(w*h, 0);
    out.write(reinterpret_cast<char*>(blank.data()), blank.size());
  }

  pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    pose_topic_, queue_size_,
    std::bind(&FlattenPointCloudToMap2D::poseCallback, this, std::placeholders::_1));
  cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
    cloud_topic_, queue_size_,
    std::bind(&FlattenPointCloudToMap2D::cloudCallback, this, std::placeholders::_1));
  rclcpp::QoS latch_qos(queue_size_); latch_qos.transient_local(); latch_qos.reliable();
  publisher_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>(
    map2d_topic_, latch_qos);

  loadGrid();
  publishFullGrid();
}

BT::NodeStatus FlattenPointCloudToMap2D::tick()
{
  if (!have_pose_ || !have_cloud_) {
    return BT::NodeStatus::RUNNING;
  }

  // center of update patch
  const double cx = current_pose_.position.x;
  const double cy = current_pose_.position.y;
  const double r2 = update_radius_m_ * update_radius_m_;

  // clear just the local patch
  const int ci = int((cx - origin_x_) / resolution_);
  const int cj = int((cy - origin_y_) / resolution_);
  const int ri = int(update_radius_m_ / resolution_);
  for (int jj = std::max(cj-ri,0); jj <= std::min(cj+ri,int(global_h_)-1); ++jj) {
    for (int ii = std::max(ci-ri,0); ii <= std::min(ci+ri,int(global_w_)-1); ++ii) {
      const double wx = origin_x_ + (ii+0.5)*resolution_;
      const double wy = origin_y_ + (jj+0.5)*resolution_;
      if ((wx-cx)*(wx-cx) + (wy-cy)*(wy-cy) <= r2) {
        grid_[jj*global_w_ + ii] = 0;
      }
    }
  }

  // iterate over pointcloud and mark any point > height as obstacle
  for (sensor_msgs::PointCloud2ConstIterator<float> it_x(*cloud_msg_, "x"),
       it_y(*cloud_msg_, "y"), it_z(*cloud_msg_, "z");
       it_x != it_x.end(); ++it_x, ++it_y, ++it_z)
  {
    const float px = *it_x, py = *it_y, pz = *it_z;
    if (!std::isfinite(px) || !std::isfinite(py) || !std::isfinite(pz)) {
      continue;
    }
    // in local patch?
    const double dx = px - cx, dy = py - cy;
    if (dx*dx + dy*dy > r2) {
      continue;
    }
    // above threshold?
    if (pz <= ground_height_thresh_) {
      continue;
    }
    // project into grid
    const int ix = int((px - origin_x_) / resolution_);
    const int iy = int((py - origin_y_) / resolution_);
    if (ix >= 0 && ix < int(global_w_) && iy >= 0 && iy < int(global_h_)) {
      grid_[iy*global_w_ + ix] = 100;
    }
  }

  publishFullGrid();
  return BT::NodeStatus::SUCCESS;
}

void FlattenPointCloudToMap2D::halt()
{
  // nothing
}

}  // namespace go2_control_cpp

BT_REGISTER_NODES(factory)
{
  factory.registerBuilder<go2_control_cpp::FlattenPointCloudToMap2D>(
    "FlattenPointCloudToMap2D",
    [](const std::string & name, const BT::NodeConfiguration & config){
      return std::make_unique<go2_control_cpp::FlattenPointCloudToMap2D>(name, config);
    });
}
