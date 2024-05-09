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
#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace smart_gap_follow
{

class SmartGapFollowNode : public rclcpp::Node
{
public:
  explicit SmartGapFollowNode(const rclcpp::NodeOptions & node_options);

  struct NodeParam
  {
    std::vector<int64_t> bev_origin{};
    double px_dist{};
    double car_width{};
    double car_length{};
    double car_back_to_drive_axle{};
    double car_back_to_steer_axle{};
    double scan_angle_increment{};
    double scan_angle_min{};
    double scan_angle_max{};
    double scan_range_min{};
    double scan_range_max{};
    double dead_end_scan_range_max{};
    int ray_step_size{};
    double min_gap_size{};
    double goal_angle{};
    double wall_follow_clearance_min{};
    double wall_follow_clearance_max{};
    double min_speed{};
    double slow_speed{};
    double max_speed{};
  };

  struct Pos
  {
    double x;
    double y;
  };

  struct Scan
  {
    std::vector<double> scan_ranges;
    std::vector<std::pair<int, int>> gap_indexes;
    std::pair<int, int> target_gap_idx;
    std::pair<Pos, Pos> target_gap_pos;
    double target_gap_length;
    double dist_to_target_gap;
    bool cross_over;
    bool deadend;
  };

private:
  // Subscriber
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_{};

  // Callback
  void onImage(const sensor_msgs::msg::Image::ConstSharedPtr msg);

  Scan getScanAtPose();
  
  void analyzeScan(Scan& scan);

  // Data Buffer
  sensor_msgs::msg::Image::ConstSharedPtr image_{};

  // Publisher
  //rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_{};
  image_transport::Publisher pub_image_;
  image_transport::Publisher pub_test_image_;
  //std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::LaserScan>> pub_grass_scan_;

  // Parameter Server
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;
  rcl_interfaces::msg::SetParametersResult onSetParam(
    const std::vector<rclcpp::Parameter> & params);

  // Parameter
  NodeParam node_param_{};

};

}  // namespace classic_grass_detection
