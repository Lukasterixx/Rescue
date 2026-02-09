#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <Eigen/Core>

class FloorRemover : public rclcpp::Node {
public:
  explicit FloorRemover(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

  // Parameters for floor detection and filtering
  double distance_threshold_;    // max distance to plane for inliers
  double axis_tolerance_deg_;    // orientation tolerance for floor normal
  int    max_iterations_;        // RANSAC max trials
  double voxel_leaf_size_;       // downsample leaf size
  double z_max_height_;          // height below which to segment floor
  double radius_max_;            // maximum radius (euclidean) to keep
};