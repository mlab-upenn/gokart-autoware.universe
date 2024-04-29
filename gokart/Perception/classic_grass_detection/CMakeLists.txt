cmake_minimum_required(VERSION 3.5)
project(classic_grass_detection)

# Dependencies
find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

ament_auto_find_build_dependencies()

# Targets
ament_auto_add_library(classic_grass_detection_node_component SHARED
  src/classic_grass_detection_node/classic_grass_detection_node.cpp
)

rclcpp_components_register_node(classic_grass_detection_node_component
  PLUGIN "classic_grass_detection::ClassicGrassDetectionNode"
  EXECUTABLE classic_grass_detection_node
)

# Package
ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
