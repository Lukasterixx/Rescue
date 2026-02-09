#include "go2_control_cpp/floor_remover.hpp"

FloorRemover::FloorRemover(const rclcpp::NodeOptions & options)
: Node("floor_remover", options)
{
  // Declare parameters
  declare_parameter<double>("distance_threshold", 0.8);
  declare_parameter<double>("axis_tolerance_deg", 8.0);
  declare_parameter<int>("max_iterations", 1000);
  declare_parameter<double>("voxel_leaf_size", 0.1);
  declare_parameter<double>("z_max_height", 0.6);
  declare_parameter<double>("radius_max", 20.0);

  // Get parameters
  get_parameter("distance_threshold", distance_threshold_);
  get_parameter("axis_tolerance_deg", axis_tolerance_deg_);
  get_parameter("max_iterations", max_iterations_);
  get_parameter("voxel_leaf_size", voxel_leaf_size_);
  get_parameter("z_max_height", z_max_height_);
  get_parameter("radius_max", radius_max_);

  // Create subscription and publisher
  sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "/point_cloud2", rclcpp::SensorDataQoS(),
    std::bind(&FloorRemover::cloudCallback, this, std::placeholders::_1));

  pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "/point_cloud3", rclcpp::QoS(10));

  RCLCPP_INFO(
    get_logger(),
    "FloorRemover initialized: z_max=%.2f, radius_max=%.2f",
    z_max_height_, radius_max_);
}

void FloorRemover::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // Convert ROS2 msg to PCL cloud
  pcl::PointCloud<pcl::PointXYZI>::Ptr input(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*msg, *input);

  // 1) Voxel downsampling for speed
  pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::VoxelGrid<pcl::PointXYZI> vg;
  vg.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
  vg.setInputCloud(input);
  vg.filter(*downsampled);

  // 2) Radius filter: keep only points within radius_max_
  pcl::PointCloud<pcl::PointXYZI>::Ptr radius_filtered(
    new pcl::PointCloud<pcl::PointXYZI>
  );
  double r2 = radius_max_ * radius_max_;
  for (const auto &pt : downsampled->points) {
    if (pt.x * pt.x + pt.y * pt.y + pt.z * pt.z <= r2) {
      radius_filtered->points.push_back(pt);
    }
  }
  radius_filtered->width = radius_filtered->points.size();
  radius_filtered->height = 1;
  radius_filtered->is_dense = true;
  downsampled = radius_filtered;

  // 3) Split into low (<z_max_height_) and high (>=z_max_height_) clouds
  pcl::PointCloud<pcl::PointXYZI>::Ptr lowz(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr highz(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PassThrough<pcl::PointXYZI> pass;
  pass.setInputCloud(downsampled);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(std::numeric_limits<float>::lowest(), static_cast<float>(z_max_height_));
  pass.filter(*lowz);
  pass.setFilterLimits(static_cast<float>(z_max_height_), std::numeric_limits<float>::max());
  pass.filter(*highz);

  // 4) Plane segmentation (RANSAC) on lowz
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::SACSegmentation<pcl::PointXYZI> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(distance_threshold_);
  seg.setAxis(Eigen::Vector3f(0.0, 0.0, 1.0));
  seg.setEpsAngle(axis_tolerance_deg_ * M_PI / 180.0);
  seg.setMaxIterations(max_iterations_);
  seg.setInputCloud(lowz);
  seg.segment(*inliers, *coefficients);

  // 5) If no plane found, republish raw cloud
  if (inliers->indices.empty()) {
    RCLCPP_WARN(get_logger(), "No floor plane found, republishing raw cloud.");
    pub_->publish(*msg);
    return;
  }

  // 6) Extract non-floor points from lowz
  pcl::PointCloud<pcl::PointXYZI>::Ptr low_filtered(
    new pcl::PointCloud<pcl::PointXYZI>
  );
  pcl::ExtractIndices<pcl::PointXYZI> extract;
  extract.setInputCloud(lowz);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*low_filtered);

  // 7) Merge low_filtered + highz
  pcl::PointCloud<pcl::PointXYZI> merged = *low_filtered + *highz;
  // create the output cloud pointer from merged data
pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(
  new pcl::PointCloud<pcl::PointXYZI>(merged)
);

  // 8) Convert back to ROS2 msg and publish
  sensor_msgs::msg::PointCloud2 out;
  pcl::toROSMsg(*output_cloud, out);
  out.header = msg->header;
  pub_->publish(out);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FloorRemover>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}