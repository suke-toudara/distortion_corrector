#include "distortion_corrector/distortion_corrector_node.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cstring>

namespace distortion_corrector
{

DistortionCorrectorNode::DistortionCorrectorNode(const rclcpp::NodeOptions & options)
: Node("distortion_corrector", options),
  shutdown_(false)
{
  // Parameters
  scan_duration_ = this->declare_parameter<double>("scan_duration", 0.1);
  max_buffer_size_ = static_cast<size_t>(this->declare_parameter<int>("max_buffer_size", 100));
  use_imu_ = this->declare_parameter<bool>("use_imu", true);
  use_odom_ = this->declare_parameter<bool>("use_odom", true);

  // Publisher
  pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "~/output/pointcloud", 10);

  // Subscribers
  pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "~/input/pointcloud", 10,
    std::bind(&DistortionCorrectorNode::pointCloudCallback, this, std::placeholders::_1));

  if (use_imu_) {
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "~/input/imu", 100,
      std::bind(&DistortionCorrectorNode::imuCallback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Using IMU for rotation correction");
  }

  if (use_odom_) {
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "~/input/odom", 100,
      std::bind(&DistortionCorrectorNode::odomCallback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Using Odometry for translation correction");
  }

  // Start processing thread (Livox-style)
  processing_thread_ = std::thread(&DistortionCorrectorNode::processingLoop, this);
}

DistortionCorrectorNode::~DistortionCorrectorNode()
{
  {
    std::lock_guard<std::mutex> lock(mtx_buffer_);
    shutdown_ = true;
  }
  sig_buffer_.notify_all();
  if (processing_thread_.joinable()) {
    processing_thread_.join();
  }
}

void DistortionCorrectorNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mtx_buffer_);
  ImuData data;
  data.time = rclcpp::Time(msg->header.stamp).seconds();
  data.orientation = Eigen::Quaterniond(
    msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
  data.angular_velocity = Eigen::Vector3d(
    msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
  imu_buffer_.push_back(data);

  while (imu_buffer_.size() > max_buffer_size_) {
    imu_buffer_.pop_front();
  }
  sig_buffer_.notify_all();
}

void DistortionCorrectorNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mtx_buffer_);

  OdomData data;
  data.time = rclcpp::Time(msg->header.stamp).seconds();
  data.position = Eigen::Vector3d(
    msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
  data.orientation = Eigen::Quaterniond(
    msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);

  odom_buffer_.push_back(data);

  while (odom_buffer_.size() > max_buffer_size_) {
    odom_buffer_.pop_front();
  }

  sig_buffer_.notify_all();
}

void DistortionCorrectorNode::pointCloudCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mtx_buffer_);

  cloud_buffer_.push_back(msg);

  // Remove old data
  while (cloud_buffer_.size() > 10) {
    cloud_buffer_.pop_front();
  }

  // Notify processing thread (Livox-style)
  sig_buffer_.notify_all();
}

void DistortionCorrectorNode::processingLoop()
{
  while (!shutdown_) {
    std::unique_lock<std::mutex> lock(mtx_buffer_);
    sig_buffer_.wait(lock, [this] {
      return shutdown_ || !cloud_buffer_.empty();
    });

    if (cloud_buffer_.empty()) continue;

    bool has_imu_data = use_imu_ && imu_buffer_.size() >= 2;
    bool has_odom_data = use_odom_ && odom_buffer_.size() >= 2;

    if ((use_imu_ && !has_imu_data) || (use_odom_ && !has_odom_data)) continue;

    auto cloud_msg = cloud_buffer_.front();
    cloud_buffer_.clear();

    lock.unlock();

    sensor_msgs::msg::PointCloud2 output;
    if (correctDistortion(cloud_msg, output)) {
      pointcloud_pub_->publish(output);
    }
  }
}



bool DistortionCorrectorNode::correctDistortion(
  const sensor_msgs::msg::PointCloud2::SharedPtr & cloud_msg,
  sensor_msgs::msg::PointCloud2 & output)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*cloud_msg, *cloud_in);

  const double ref_time = rclcpp::Time(cloud_msg->header.stamp).seconds();

  // Get reference pose (at scan end time)
  Eigen::Quaterniond ref_rotation;
  Eigen::Vector3d ref_translation;

  if (!interpolateRotation(ref_time, ref_rotation)) {
    if (use_imu_) return false;
  }

  if (!interpolateTranslation(ref_time, ref_translation)) {
    if (use_odom_) return false;
  }

  const size_t num_points = cloud_in->points.size();
  cloud_out->points.reserve(num_points);
  cloud_out->header = cloud_in->header;
  cloud_out->is_dense = cloud_in->is_dense;

  // Check if timestamp field exists in the input point cloud
  bool has_timestamp = false;
  size_t timestamp_offset = 0;
  for (const auto & field : cloud_msg->fields) {
    if (field.name == "timestamp") {
      has_timestamp = true;
      timestamp_offset = field.offset;
      break;
    }
  }

  // Process each point
  for (size_t i = 0; i < num_points; ++i) {
    const auto & pt_in = cloud_in->points[i];

    // Skip invalid points
    if (!std::isfinite(pt_in.x) || !std::isfinite(pt_in.y) || !std::isfinite(pt_in.z)) {
      continue;
    }

    double point_time;
    if (has_timestamp) {
      // Extract timestamp from the original ROS message raw data
      const uint8_t* point_data = &cloud_msg->data[i * cloud_msg->point_step + timestamp_offset];
      double timestamp_seconds;
      
      // Assume timestamp is stored as double (8 bytes)
      std::memcpy(&timestamp_seconds, point_data, sizeof(double));
      point_time = timestamp_seconds;
    } else {
      // Calculate time based on point index when no timestamp field is available
      const double point_ratio = static_cast<double>(i) / static_cast<double>(num_points);
      point_time = ref_time - scan_duration_ + point_ratio * scan_duration_;
    }

    Eigen::Quaterniond point_rotation;
    Eigen::Vector3d point_translation;

    if (!interpolateRotation(point_time, point_rotation)) {
      point_rotation = ref_rotation;
    }

    if (!interpolateTranslation(point_time, point_translation)) {
      point_translation = ref_translation;
    }

    // Point in sensor frame
    Eigen::Vector3d pt(pt_in.x, pt_in.y, pt_in.z);

    // Correction: remove motion between point time and reference time
    // Step 1: Transform to world frame at point time
    Eigen::Vector3d pt_world = point_rotation * pt + point_translation;

    // Step 2: Transform back to sensor frame at reference time
    Eigen::Vector3d pt_corrected = ref_rotation.inverse() * (pt_world - ref_translation);

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
  output.header = cloud_msg->header;
  return true;
}

bool DistortionCorrectorNode::interpolateRotation(
  double time, Eigen::Quaterniond & rotation)
{
  if (!use_imu_ || imu_buffer_.size() < 2) {
    rotation = Eigen::Quaterniond::Identity();
    return false;
  }

  std::lock_guard<std::mutex> lock(mtx_buffer_);

  // Find surrounding IMU data
  ImuData before, after;
  bool found = false;

  // 対象の点群の時刻に対して前後のIMUデータを探索
  for (size_t i = 0; i < imu_buffer_.size() - 1; ++i) {
    if (imu_buffer_[i].time <= time && time <= imu_buffer_[i + 1].time) {
      before = imu_buffer_[i];
      after = imu_buffer_[i + 1];
      found = true;
      break;
    }
  }

  if (!found) {
    // Use closest
    auto closest = std::min_element(
      imu_buffer_.begin(), imu_buffer_.end(),
      [time](const ImuData & a, const ImuData & b) {
        return std::abs(a.time - time) < std::abs(b.time - time);
      });

    if (closest != imu_buffer_.end()) {
      rotation = closest->orientation;
      return true;
    }
    return false;
  }

  // SLERP interpolation
  double ratio = (time - before.time) / (after.time - before.time);
  rotation = before.orientation.slerp(ratio, after.orientation);
  return true;
}


bool DistortionCorrectorNode::interpolateTranslation(
  double time, Eigen::Vector3d & translation)
{
  if (!use_odom_ || odom_buffer_.size() < 2) {
    translation = Eigen::Vector3d::Zero();
    return false;
  }

  std::lock_guard<std::mutex> lock(mtx_buffer_);

  // Find surrounding Odom data
  OdomData before, after;
  bool found = false;

  for (size_t i = 0; i < odom_buffer_.size() - 1; ++i) {
    if (odom_buffer_[i].time <= time && time <= odom_buffer_[i + 1].time) {
      before = odom_buffer_[i];
      after = odom_buffer_[i + 1];
      found = true;
      break;
    }
  }

  if (!found) {
    // Use closest
    auto closest = std::min_element(
      odom_buffer_.begin(), odom_buffer_.end(),
      [time](const OdomData & a, const OdomData & b) {
        return std::abs(a.time - time) < std::abs(b.time - time);
      });

    if (closest != odom_buffer_.end()) {
      translation = closest->position;
      return true;
    }
    return false;
  }

  // Linear interpolation
  double ratio = (time - before.time) / (after.time - before.time);
  translation = before.position + ratio * (after.position - before.position);
  return true;
}


}  // namespace distortion_corrector

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(distortion_corrector::DistortionCorrectorNode)
