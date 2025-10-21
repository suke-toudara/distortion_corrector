#include <rclcpp/rclcpp.hpp>
#include "distortion_corrector/distortion_corrector_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<distortion_corrector::DistortionCorrectorNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
