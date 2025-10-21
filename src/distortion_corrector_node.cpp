#include "distortion_corrector/distortion_corrector_node.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace distortion_corrector
{

DistortionCorrectorNode::DistortionCorrectorNode(const rclcpp::NodeOptions & options)
: Node("distortion_corrector", options)
{
  // Parameters
  base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
  use_data_source_ = this->declare_parameter<std::string>("use_data_source", "imu");
  queue_size_ = this->declare_parameter<double>("queue_size", 2.0);
  max_queue_size_ = static_cast<size_t>(this->declare_parameter<int>("max_queue_size", 200));

  // Initialize TF
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Publishers
  pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "~/output/pointcloud", 10);

  // Subscribers
  pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "~/input/pointcloud", 10,
    std::bind(&DistortionCorrectorNode::pointCloudCallback, this, std::placeholders::_1));

  if (use_data_source_ == "imu") {
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "~/input/imu", 100,
      std::bind(&DistortionCorrectorNode::imuCallback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Using IMU for distortion correction");
  } else if (use_data_source_ == "odom") {
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "~/input/odom", 100,
      std::bind(&DistortionCorrectorNode::odomCallback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Using Odometry for distortion correction");
  } else {
    RCLCPP_ERROR(this->get_logger(), "Invalid use_data_source: %s (must be 'imu' or 'odom')",
                 use_data_source_.c_str());
  }

  RCLCPP_INFO(this->get_logger(), "Distortion Corrector Node initialized");
}

void DistortionCorrectorNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  imu_queue_.push_back(msg);

  // Remove old data
  const auto time_thresh = rclcpp::Time(msg->header.stamp) - rclcpp::Duration::from_seconds(queue_size_);
  while (!imu_queue_.empty()) {
    if (rclcpp::Time(imu_queue_.front()->header.stamp) < time_thresh) {
      imu_queue_.pop_front();
    } else {
      break;
    }
  }

  // Limit queue size
  while (imu_queue_.size() > max_queue_size_) {
    imu_queue_.pop_front();
  }
}

void DistortionCorrectorNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  odom_queue_.push_back(msg);

  // Remove old data
  const auto time_thresh = rclcpp::Time(msg->header.stamp) - rclcpp::Duration::from_seconds(queue_size_);
  while (!odom_queue_.empty()) {
    if (rclcpp::Time(odom_queue_.front()->header.stamp) < time_thresh) {
      odom_queue_.pop_front();
    } else {
      break;
    }
  }

  // Limit queue size
  while (odom_queue_.size() > max_queue_size_) {
    odom_queue_.pop_front();
  }
}

void DistortionCorrectorNode::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  sensor_msgs::msg::PointCloud2 output;

  if (undistortPointCloud(*msg, output)) {
    pointcloud_pub_->publish(output);
  } else {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Failed to undistort point cloud");
  }
}

bool DistortionCorrectorNode::getTransform(
  const std::string & target_frame,
  const std::string & source_frame,
  const rclcpp::Time & time,
  geometry_msgs::msg::TransformStamped & transform)
{
  try {
    transform = tf_buffer_->lookupTransform(
      target_frame, source_frame, time, rclcpp::Duration::from_seconds(0.1));
    return true;
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Could not transform: %s", ex.what());
    return false;
  }
}

bool DistortionCorrectorNode::undistortPointCloud(
  const sensor_msgs::msg::PointCloud2 & input,
  sensor_msgs::msg::PointCloud2 & output)
{
  // Check if we have enough data
  if (use_data_source_ == "imu" && imu_queue_.size() < 2) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Not enough IMU data for correction");
    return false;
  }
  if (use_data_source_ == "odom" && odom_queue_.size() < 2) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Not enough Odom data for correction");
    return false;
  }

  // Convert to PCL
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(input, *cloud_in);

  // Get reference transform (latest time)
  const rclcpp::Time ref_time = rclcpp::Time(input.header.stamp);
  geometry_msgs::msg::TransformStamped ref_transform;

  if (!getTransform(base_frame_, input.header.frame_id, ref_time, ref_transform)) {
    return false;
  }

  // Calculate scan duration (assume uniform point distribution)
  const double scan_duration = 0.1;  // 100ms typical for most LiDARs
  const size_t num_points = cloud_in->points.size();

  cloud_out->points.reserve(num_points);
  cloud_out->header = cloud_in->header;
  cloud_out->is_dense = cloud_in->is_dense;

  // Process each point
  for (size_t i = 0; i < num_points; ++i) {
    const auto & pt_in = cloud_in->points[i];

    // Skip invalid points
    if (!std::isfinite(pt_in.x) || !std::isfinite(pt_in.y) || !std::isfinite(pt_in.z)) {
      continue;
    }

    // Calculate point timestamp (linear interpolation)
    const double point_time_offset = (static_cast<double>(i) / num_points) * scan_duration;
    const rclcpp::Time point_time = ref_time - rclcpp::Duration::from_seconds(scan_duration - point_time_offset);

    // Get transform for this point's time
    geometry_msgs::msg::TransformStamped point_transform;
    if (!getTransform(base_frame_, input.header.frame_id, point_time, point_transform)) {
      continue;
    }

    // Convert point to Eigen
    Eigen::Vector3d pt_eigen(pt_in.x, pt_in.y, pt_in.z);

    // Apply point transform
    Eigen::Affine3d point_affine = tf2::transformToEigen(point_transform);
    Eigen::Vector3d pt_transformed = point_affine * pt_eigen;

    // Apply inverse reference transform to bring back to sensor frame at reference time
    Eigen::Affine3d ref_affine = tf2::transformToEigen(ref_transform);
    Eigen::Vector3d pt_corrected = ref_affine.inverse() * pt_transformed;

    // Add corrected point
    pcl::PointXYZI pt_out;
    pt_out.x = static_cast<float>(pt_corrected.x());
    pt_out.y = static_cast<float>(pt_corrected.y());
    pt_out.z = static_cast<float>(pt_corrected.z());
    pt_out.intensity = pt_in.intensity;
    cloud_out->points.push_back(pt_out);
  }

  // Convert back to ROS message
  pcl::toROSMsg(*cloud_out, output);
  output.header = input.header;

  return true;
}

}  // namespace distortion_corrector

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(distortion_corrector::DistortionCorrectorNode)
