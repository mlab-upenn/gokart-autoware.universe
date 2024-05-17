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
#include "image_transport/image_transport.hpp"
#include "image_transport/subscriber_filter.hpp"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"
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
    bool simulation{};
    int cam_id{};
    int bev_width{};
    int bev_height{};
    std::vector<int64_t> bev_origin{};
    double px_dist{};
    double grass_b_weight{};
    double grass_g_weight{};
    double cone_b_weight{};
    double cone_r_weight{};
    int grass_thres{};
    int cone_thres{};
    double cone_base_ratio{};
    std::vector<int64_t> img_bot_left{};
    std::vector<int64_t> img_top_left{};
    std::vector<int64_t> img_top_right{};
    std::vector<int64_t> img_bot_right{};
    std::vector<double> world_bot_left{};
    std::vector<double> world_top_left{};
    std::vector<double> world_top_right{};
    std::vector<double> world_bot_right{};
    double scan_angle_increment{};
    double scan_angle_min{};
    double scan_angle_max{};
    double scan_range_min{};
    double scan_range_max{};
    int ray_step_size{};
  };

private:
  // Subscriber
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_cam_image_{};

  // Callback
  void onImage(const sensor_msgs::msg::Image::ConstSharedPtr msg);

  // Data Buffer
  sensor_msgs::msg::Image::ConstSharedPtr cam_image_{};
  rclcpp::TimerBase::SharedPtr timer_;

  void camImageDriver();

  // Publisher
  image_transport::Publisher pub_cam_;
  image_transport::Publisher pub_cam_bev_;
  image_transport::Publisher pub_track_bev_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::LaserScan>> pub_track_scan_;

  // Parameter Server
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;
  rcl_interfaces::msg::SetParametersResult onSetParam(
    const std::vector<rclcpp::Parameter> & params);

  // Parameter
  NodeParam node_param_{};

  cv::VideoCapture cap;
};

}  // namespace classic_grass_detection
