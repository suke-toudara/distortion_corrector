#include "distortion_corrector/distortion_corrector_node.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace
{
// Linear interpolation for translation
Eigen::Vector3d lerpVector(
  const Eigen::Vector3d & start, const Eigen::Vector3d & end, double ratio)
{
  return start + (end - start) * ratio;
}

// Spherical linear interpolation for rotation
Eigen::Quaterniond slerpQuaternion(
  const Eigen::Quaterniond & start, const Eigen::Quaterniond & end, double ratio)
{
  return start.slerp(ratio, end);
}
}  // namespace

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

bool DistortionCorrectorNode::interpolatePose(
  const rclcpp::Time & target_time,
  Eigen::Affine3d & pose)
{
  if (use_data_source_ == "imu") {
    return interpolatePoseFromIMU(target_time, pose);
  } else if (use_data_source_ == "odom") {
    return interpolatePoseFromOdom(target_time, pose);
  }
  return false;
}

bool DistortionCorrectorNode::interpolatePoseFromIMU(
  const rclcpp::Time & target_time,
  Eigen::Affine3d & pose)
{
  if (imu_queue_.size() < 2) {
    return false;
  }

  // Find two IMU messages surrounding the target time
  sensor_msgs::msg::Imu::SharedPtr imu_before = nullptr;
  sensor_msgs::msg::Imu::SharedPtr imu_after = nullptr;

  for (size_t i = 0; i < imu_queue_.size() - 1; ++i) {
    const auto t_current = rclcpp::Time(imu_queue_[i]->header.stamp);
    const auto t_next = rclcpp::Time(imu_queue_[i + 1]->header.stamp);

    if (t_current <= target_time && target_time <= t_next) {
      imu_before = imu_queue_[i];
      imu_after = imu_queue_[i + 1];
      break;
    }
  }

  // If exact match not found, use closest available
  if (!imu_before || !imu_after) {
    const auto closest = std::min_element(
      imu_queue_.begin(), imu_queue_.end(),
      [&target_time](const auto & a, const auto & b) {
        return std::abs((rclcpp::Time(a->header.stamp) - target_time).seconds()) <
               std::abs((rclcpp::Time(b->header.stamp) - target_time).seconds());
      });

    if (closest != imu_queue_.end()) {
      const auto & imu = *closest;
      Eigen::Quaterniond q(
        imu->orientation.w, imu->orientation.x, imu->orientation.y, imu->orientation.z);
      pose = Eigen::Affine3d::Identity();
      pose.linear() = q.normalized().toRotationMatrix();
      return true;
    }
    return false;
  }

  // Interpolate orientation
  const double t_before = rclcpp::Time(imu_before->header.stamp).seconds();
  const double t_after = rclcpp::Time(imu_after->header.stamp).seconds();
  const double t_target = target_time.seconds();
  const double ratio = (t_target - t_before) / (t_after - t_before);

  Eigen::Quaterniond q_before(
    imu_before->orientation.w, imu_before->orientation.x,
    imu_before->orientation.y, imu_before->orientation.z);
  Eigen::Quaterniond q_after(
    imu_after->orientation.w, imu_after->orientation.x,
    imu_after->orientation.y, imu_after->orientation.z);

  // SLERP for smooth rotation interpolation
  Eigen::Quaterniond q_interpolated = slerpQuaternion(q_before, q_after, ratio);

  // IMU provides only orientation, no translation
  pose = Eigen::Affine3d::Identity();
  pose.linear() = q_interpolated.normalized().toRotationMatrix();

  return true;
}

bool DistortionCorrectorNode::interpolatePoseFromOdom(
  const rclcpp::Time & target_time,
  Eigen::Affine3d & pose)
{
  if (odom_queue_.size() < 2) {
    return false;
  }

  // Find two Odom messages surrounding the target time
  nav_msgs::msg::Odometry::SharedPtr odom_before = nullptr;
  nav_msgs::msg::Odometry::SharedPtr odom_after = nullptr;

  for (size_t i = 0; i < odom_queue_.size() - 1; ++i) {
    const auto t_current = rclcpp::Time(odom_queue_[i]->header.stamp);
    const auto t_next = rclcpp::Time(odom_queue_[i + 1]->header.stamp);

    if (t_current <= target_time && target_time <= t_next) {
      odom_before = odom_queue_[i];
      odom_after = odom_queue_[i + 1];
      break;
    }
  }

  // If exact match not found, use closest available
  if (!odom_before || !odom_after) {
    const auto closest = std::min_element(
      odom_queue_.begin(), odom_queue_.end(),
      [&target_time](const auto & a, const auto & b) {
        return std::abs((rclcpp::Time(a->header.stamp) - target_time).seconds()) <
               std::abs((rclcpp::Time(b->header.stamp) - target_time).seconds());
      });

    if (closest != odom_queue_.end()) {
      const auto & odom = *closest;
      Eigen::Vector3d trans(
        odom->pose.pose.position.x, odom->pose.pose.position.y, odom->pose.pose.position.z);
      Eigen::Quaterniond q(
        odom->pose.pose.orientation.w, odom->pose.pose.orientation.x,
        odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);

      pose = Eigen::Affine3d::Identity();
      pose.translation() = trans;
      pose.linear() = q.normalized().toRotationMatrix();
      return true;
    }
    return false;
  }

  // Interpolate pose
  const double t_before = rclcpp::Time(odom_before->header.stamp).seconds();
  const double t_after = rclcpp::Time(odom_after->header.stamp).seconds();
  const double t_target = target_time.seconds();
  const double ratio = (t_target - t_before) / (t_after - t_before);

  // Interpolate translation (linear)
  Eigen::Vector3d trans_before(
    odom_before->pose.pose.position.x,
    odom_before->pose.pose.position.y,
    odom_before->pose.pose.position.z);
  Eigen::Vector3d trans_after(
    odom_after->pose.pose.position.x,
    odom_after->pose.pose.position.y,
    odom_after->pose.pose.position.z);
  Eigen::Vector3d trans_interpolated = lerpVector(trans_before, trans_after, ratio);

  // Interpolate rotation (SLERP)
  Eigen::Quaterniond q_before(
    odom_before->pose.pose.orientation.w, odom_before->pose.pose.orientation.x,
    odom_before->pose.pose.orientation.y, odom_before->pose.pose.orientation.z);
  Eigen::Quaterniond q_after(
    odom_after->pose.pose.orientation.w, odom_after->pose.pose.orientation.x,
    odom_after->pose.pose.orientation.y, odom_after->pose.pose.orientation.z);
  Eigen::Quaterniond q_interpolated = slerpQuaternion(q_before, q_after, ratio);

  // Compose pose
  pose = Eigen::Affine3d::Identity();
  pose.translation() = trans_interpolated;
  pose.linear() = q_interpolated.normalized().toRotationMatrix();

  return true;
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

  // Reference time (scan end time)
  const rclcpp::Time ref_time = rclcpp::Time(input.header.stamp);

  // Get reference pose at scan end time
  Eigen::Affine3d ref_pose;
  if (!interpolatePose(ref_time, ref_pose)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Failed to get reference pose");
    return false;
  }

  // Calculate scan duration (100ms typical for most LiDARs)
  const double scan_duration = 0.1;
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

    // Calculate point timestamp (linear interpolation from scan start to end)
    const double point_time_offset = (static_cast<double>(i) / num_points) * scan_duration;
    const rclcpp::Time point_time = ref_time - rclcpp::Duration::from_seconds(scan_duration - point_time_offset);

    // Get pose at this point's capture time
    Eigen::Affine3d point_pose;
    if (!interpolatePose(point_time, point_pose)) {
      // If interpolation fails, keep original point
      pcl::PointXYZI pt_out = pt_in;
      cloud_out->points.push_back(pt_out);
      continue;
    }

    // Transform point:
    // 1. Point is in sensor frame at point_time
    // 2. Calculate relative transform from point_time to ref_time
    // 3. Apply inverse transformation to "undo" the motion

    Eigen::Vector3d pt_eigen(pt_in.x, pt_in.y, pt_in.z);

    // Relative transformation: from point capture time to reference time
    // corrected_point = ref_pose^-1 * point_pose * original_point
    Eigen::Affine3d relative_transform = ref_pose.inverse() * point_pose;
    Eigen::Vector3d pt_corrected = relative_transform * pt_eigen;

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
