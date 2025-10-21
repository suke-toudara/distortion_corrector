cmake_minimum_required(VERSION 3.8)
project(distortion_corrector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)

# Include directories
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

# Component library
add_library(distortion_corrector_component SHARED
  src/distortion_corrector_node.cpp
)

ament_target_dependencies(distortion_corrector_component
  rclcpp
  rclcpp_components
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  tf2_sensor_msgs
  pcl_conversions
  pcl_ros
)

target_link_libraries(distortion_corrector_component
  ${PCL_LIBRARIES}
)

rclcpp_components_register_node(distortion_corrector_component
  PLUGIN "distortion_corrector::DistortionCorrectorNode"
  EXECUTABLE distortion_corrector_node
)

# Standalone executable
add_executable(distortion_corrector_main
  src/distortion_corrector_main.cpp
)

target_link_libraries(distortion_corrector_main
  distortion_corrector_component
)

# Install
install(TARGETS
  distortion_corrector_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  distortion_corrector_main
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
