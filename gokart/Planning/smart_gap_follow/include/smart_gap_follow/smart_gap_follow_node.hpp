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
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "image_transport/image_transport.hpp"
#include "image_transport/subscriber_filter.hpp"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace smart_gap_follow
{
/*
using geometry_msgs::msg::Pose;
using ackermann_msgs::msg::AckermannDriveStamped;
using geometry_msgs::msg::Point;
using sensor_msgs::msg::LaserScan;
using std::vector;
using std::pair;
*/
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
    double steering_speed{};
    double range_thresh_min{};
    double range_thresh_max{};
    double min_gap_size{};
    double goal_angle{};
    double wall_follow_clearance_min{};
    double wall_follow_clearance_max{};
    double min_speed{};
    double slow_speed{};
    double max_speed{};
  };
  /*
  struct Pose
  {
    int x_px;
    int y_px;
    double x_world;
    double y_world;
    double orientation;
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
  */
private:
  // Subscriber
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_track_scan_{};
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_lidar_scan_{};
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_cam_image_{};
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_track_bev_{};

  // Callback
  void onTrackScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg);
  void onLidarScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg);
  void onCamImage(const sensor_msgs::msg::Image::ConstSharedPtr msg);
  void onTrackBev(const sensor_msgs::msg::Image::ConstSharedPtr msg);

  // Data Buffer
  sensor_msgs::msg::LaserScan::ConstSharedPtr track_scan_{};
  sensor_msgs::msg::LaserScan::ConstSharedPtr lidar_scan_{};
  sensor_msgs::msg::Image::ConstSharedPtr cam_image_{};
  sensor_msgs::msg::Image::ConstSharedPtr track_bev_{};

  // Publisher
  //rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_{};
  image_transport::Publisher pub_planning_cam_;
  image_transport::Publisher pub_planning_bev_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr pub_drive_;
  //std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::LaserScan>> pub_grass_scan_;

  // Parameter Server
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;
  rcl_interfaces::msg::SetParametersResult onSetParam(
    const std::vector<rclcpp::Parameter> & params);

  sensor_msgs::msg::LaserScan merge_scan(const sensor_msgs::msg::LaserScan scan_1,
    const sensor_msgs::msg::LaserScan scan_2);
  void findGap(const sensor_msgs::msg::LaserScan scan);
  void findTargetGap(const sensor_msgs::msg::LaserScan scan);
  void calcMotionCmd(const sensor_msgs::msg::LaserScan scan);

  // Parameter
  NodeParam node_param_{};

  sensor_msgs::msg::LaserScan track_scan;
  sensor_msgs::msg::LaserScan lidar_scan;
  sensor_msgs::msg::LaserScan merged_scan;
  double range_thresh;
  std::vector<std::pair<int, int>> gap_indices;
  bool deadend;
  std::pair<int, int> target_gap_indices;
  std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point> target_gap;

};

}  // namespace classic_grass_detection
