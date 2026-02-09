#include <memory>
#include <cmath>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

class InitialPoseNode : public rclcpp::Node
{
public:
  InitialPoseNode()
  : Node("initial_pose_node"), done_(false)
  {
    // declare parameters with defaults
    declare_parameter<std::string>("input_topic", "/point_cloud2");
    declare_parameter<double>("z_height", 0.4);
    declare_parameter<double>("seg_distance_threshold", 0.1);
    declare_parameter<double>("seg_axis_tolerance_deg", 10.0);
    declare_parameter<int>("seg_max_iterations", 500);

    // read them back
    get_parameter("input_topic",            input_topic_);
    get_parameter("z_height",               z_height_);
    get_parameter("seg_distance_threshold", seg_distance_threshold_);
    get_parameter("seg_axis_tolerance_deg", seg_axis_tolerance_deg_);
    get_parameter("seg_max_iterations",     seg_max_iterations_);

    // publisher for the computed initial pose
    pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/initial_pose", rclcpp::QoS(1));

    // single‐shot subscription
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&InitialPoseNode::cloudCb, this, std::placeholders::_1));
  }

private:
  void cloudCb(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (done_) {
      return;
    }

    RCLCPP_INFO(get_logger(), "Received first point cloud, computing floor pose...");

    // convert to PCL
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *pc);

    // RANSAC plane fitting
    pcl::ModelCoefficients::Ptr coeffs(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(seg_distance_threshold_);
    seg.setAxis(Eigen::Vector3f(0,0,1));
    seg.setEpsAngle(seg_axis_tolerance_deg_ * M_PI/180.0);
    seg.setMaxIterations(seg_max_iterations_);
    seg.setInputCloud(pc);
    seg.segment(*inliers, *coeffs);

    // default normal = up
    Eigen::Vector3f normal(0,0,1);
    if (coeffs->values.size() >= 3) {
      normal = Eigen::Vector3f(
        coeffs->values[0],
        coeffs->values[1],
        coeffs->values[2]
      ).normalized();
      if (normal.dot(Eigen::Vector3f(0,0,1)) < 0) {
        normal = -normal;
      }
    }

    // compute quaternion rotating normal → Z
    Eigen::Vector3f axis = normal.cross(Eigen::Vector3f(0,0,1));
    double angle = std::acos(normal.dot(Eigen::Vector3f(0,0,1)));
    tf2::Quaternion q;
    if (axis.norm() < 1e-6 || std::abs(angle) < 1e-3) {
      q.setRPY(0,0,0);
    } else {
      axis.normalize();
      q.setRotation(
        tf2::Vector3(axis.x(), axis.y(), axis.z()),
        angle
      );
    }

    // optional: log roll/pitch/yaw
    {
      tf2::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll,pitch,yaw);
      RCLCPP_INFO(
        get_logger(),
        "Applying floor correction RPY = [%.2f°, %.2f°, %.2f°] ////////////////////////////////////////////",
        roll*180.0/M_PI, pitch*180.0/M_PI, yaw*180.0/M_PI
      );
    }

    // publish the PoseStamped
    geometry_msgs::msg::PoseStamped ip;
    ip.header = msg->header;
    ip.header.frame_id = "map";
    ip.pose.position.x = 0.0;
    ip.pose.position.y = 0.0;
    ip.pose.position.z = z_height_;
    ip.pose.orientation.x = q.x();
    ip.pose.orientation.y = q.y();
    ip.pose.orientation.z = q.z();
    ip.pose.orientation.w = q.w();
    pub_->publish(ip);

    done_ = true;
    // now exit
    rclcpp::shutdown();
  }

  // members
  bool done_;
  std::string input_topic_;
  double z_height_, seg_distance_threshold_, seg_axis_tolerance_deg_;
  int seg_max_iterations_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<InitialPoseNode>();
  rclcpp::spin(node);
  return 0;
}