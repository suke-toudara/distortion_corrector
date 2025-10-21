#ifndef DISTORTION_CORRECTOR__DISTORTION_CORRECTOR_NODE_HPP_
#define DISTORTION_CORRECTOR__DISTORTION_CORRECTOR_NODE_HPP_

#include <deque>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

namespace distortion_corrector
{

class DistortionCorrectorNode : public rclcpp::Node
{
public:
  explicit DistortionCorrectorNode(const rclcpp::NodeOptions & options);

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  bool undistortPointCloud(
    const sensor_msgs::msg::PointCloud2 & input,
    sensor_msgs::msg::PointCloud2 & output);

  bool getTransform(
    const std::string & target_frame,
    const std::string & source_frame,
    const rclcpp::Time & time,
    geometry_msgs::msg::TransformStamped & transform);

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // Publisher
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // IMU/Odom data queue
  std::deque<sensor_msgs::msg::Imu::SharedPtr> imu_queue_;
  std::deque<nav_msgs::msg::Odometry::SharedPtr> odom_queue_;

  // Parameters
  std::string base_frame_;
  std::string use_data_source_;  // "imu" or "odom"
  double queue_size_;
  size_t max_queue_size_;
};

}  // namespace distortion_corrector

#endif  // DISTORTION_CORRECTOR__DISTORTION_CORRECTOR_NODE_HPP_
