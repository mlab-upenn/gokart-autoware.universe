cmake_minimum_required(VERSION 3.5)
project(smart_gap_follow)

# Dependencies
find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ackermann_msgs REQUIRED)

ament_auto_find_build_dependencies()

# Targets
ament_auto_add_library(smart_gap_follow_node_component SHARED
  src/smart_gap_follow_node/smart_gap_follow_node.cpp
)

rclcpp_components_register_node(smart_gap_follow_node_component
  PLUGIN "smart_gap_follow::SmartGapFollowNode"
  EXECUTABLE smart_gap_follow_node
)

# Package
ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
