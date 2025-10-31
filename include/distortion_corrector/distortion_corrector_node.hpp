#ifndef DISTORTION_CORRECTOR__DISTORTION_CORRECTOR_NODE_HPP_
#define DISTORTION_CORRECTOR__DISTORTION_CORRECTOR_NODE_HPP_

#include <deque>
#include <memory>
#include <string>
#include <thread>
#include <mutex>
#include <condition_variable>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace distortion_corrector
{

struct ImuData
{
  double time;
  Eigen::Quaterniond orientation;
  Eigen::Vector3d angular_velocity;
};

struct OdomData
{
  double time;
  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;
};

class DistortionCorrectorNode : public rclcpp::Node
{
public:
  explicit DistortionCorrectorNode(const rclcpp::NodeOptions & options);
  ~DistortionCorrectorNode();

private:
  // Callback functions
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  // Processing thread
  void processingLoop();

  // Distortion correction
  bool correctDistortion(
    const sensor_msgs::msg::PointCloud2::SharedPtr & cloud_msg,
    sensor_msgs::msg::PointCloud2 & output);

  // Interpolation functions
  bool interpolateRotation(double time, Eigen::Quaterniond & rotation);
  bool interpolateTranslation(double time, Eigen::Vector3d & translation);

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // Publisher
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;

  // Data buffers
  std::deque<sensor_msgs::msg::PointCloud2::SharedPtr> cloud_buffer_;
  std::deque<ImuData> imu_buffer_;
  std::deque<OdomData> odom_buffer_;

  // Synchronization
  std::mutex mtx_buffer_;
  std::condition_variable sig_buffer_;
  std::thread processing_thread_;
  bool shutdown_;

  // Parameters
  double scan_duration_;
  size_t max_buffer_size_;
  bool use_imu_;
  bool use_odom_;
};

}  // namespace distortion_corrector

#endif  // DISTORTION_CORRECTOR__DISTORTION_CORRECTOR_NODE_HPP_
