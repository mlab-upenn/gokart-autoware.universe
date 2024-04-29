// Copyright 2023 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace classic_grass_detection
{

class ClassicGrassDetectionNode : public rclcpp::Node
{
public:
  explicit ClassicGrassDetectionNode(const rclcpp::NodeOptions & node_options);

  struct NodeParam
  {
    int bev_width{};
    int bev_height{};
    int bev_origin_x{};
    int bev_origin_y{};
    double px_dist{};
  };

private:
  // Subscriber
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_{};

  // Callback
  void onImage(const sensor_msgs::msg::Image::ConstSharedPtr msg);

  // Data Buffer
  sensor_msgs::msg::Image::ConstSharedPtr image_{};

  // Publisher
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_{};

  // Parameter Server
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;
  rcl_interfaces::msg::SetParametersResult onSetParam(
    const std::vector<rclcpp::Parameter> & params);

  // Parameter
  NodeParam node_param_{};
};

}  // namespace classic_grass_detection
