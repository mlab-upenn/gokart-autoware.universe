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

#include "smart_gap_follow/smart_gap_follow_node.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace
{
template <class T>
bool update_param(
  const std::vector<rclcpp::Parameter> & params, const std::string & name, T & value)
{
  const auto itr = std::find_if(
    params.cbegin(), params.cend(),
    [&name](const rclcpp::Parameter & p) { return p.get_name() == name; });

  // Not found
  if (itr == params.cend()) {
    return false;
  }

  value = itr->template get_value<T>();
  return true;
}

}  // namespace

namespace smart_gap_follow
{

SmartGapFollowNode::SmartGapFollowNode(const rclcpp::NodeOptions & node_options)
: Node("smart_gap_follow", node_options)
{
  // Parameter Server
  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&SmartGapFollowNode::onSetParam, this, std::placeholders::_1));

  // Node Parameter
  node_param_.bev_origin = declare_parameter<std::vector<int64_t>>("bev_origin");
  node_param_.px_dist = declare_parameter<double>("px_dist");
  node_param_.car_width = declare_parameter<double>("car_width");
  node_param_.car_length = declare_parameter<double>("car_length");
  node_param_.car_back_to_drive_axle = declare_parameter<double>("car_back_to_drive_axle");
  node_param_.car_back_to_steer_axle = declare_parameter<double>("car_back_to_steer_axle");
  node_param_.scan_angle_increment = declare_parameter<double>("scan_angle_increment");
  node_param_.scan_angle_min = declare_parameter<double>("scan_angle_min");
  node_param_.scan_angle_max = declare_parameter<double>("scan_angle_max");
  node_param_.scan_range_min = declare_parameter<double>("scan_range_min");
  node_param_.scan_range_max = declare_parameter<double>("scan_range_max");
  node_param_.dead_end_scan_range_max = declare_parameter<double>("dead_end_scan_range_max");
  node_param_.ray_step_size = declare_parameter<int>("ray_step_size");
  node_param_.min_gap_size = declare_parameter<double>("min_gap_size");
  node_param_.goal_angle = declare_parameter<double>("goal_angle");
  node_param_.wall_follow_clearance_min = declare_parameter<double>("wall_follow_clearance_min");
  node_param_.wall_follow_clearance_max = declare_parameter<double>("wall_follow_clearance_max");
  node_param_.scan_angle_increment = declare_parameter<double>("scan_angle_increment");
  node_param_.min_speed = declare_parameter<double>("min_speed");
  node_param_.slow_speed = declare_parameter<double>("slow_speed");
  node_param_.max_speed = declare_parameter<double>("max_speed");

  // Subscriber
  sub_image_ = create_subscription<sensor_msgs::msg::Image>(
    "~/input/image", rclcpp::QoS{1},
    std::bind(&SmartGapFollowNode::onImage, this, std::placeholders::_1));

  // Publisher
  //pub_image_ = create_publisher<sensor_msgs::msg::Image>("~/output/image", 1);
  pub_image_ = image_transport::create_publisher(this, "~/out/image");
  pub_test_image_ = image_transport::create_publisher(this, "~/test/image");
  //pub_planning_bev = this->create_publisher<sensor_msgs::msg::LaserScan>(
  //  "~/out/grass_scan", rclcpp::SensorDataQoS());
}

void SmartGapFollowNode::onImage(const sensor_msgs::msg::Image::ConstSharedPtr image_)
{/*
  std::vector<int64_t> bev_origin = node_param_.bev_origin;
  double px_dist = node_param_.px_dist;
  double car_width = node_param_.car_width;
  double car_length = node_param_.car_length;
  double car_back_to_drive_axle = node_param_.car_back_to_drive_axle;
  double car_back_to_steer_axle = node_param_.car_back_to_steer_axle;
  double scan_angle_increment = node_param_.scan_angle_increment;
  double scan_angle_min = node_param_.scan_angle_min;
  double scan_angle_max = node_param_.scan_angle_max;
  double scan_range_min = node_param_.scan_range_min;
  double scan_range_max = node_param_.scan_range_max;
  double dead_end_scan_range_max = node_param_.dead_end_scan_range_max;
  int ray_dist_step = node_param_.ray_dist_step;
  double min_gap_size = node_param_.min_gap_size;
  double goal_angle = node_param_.goal_angle;
  double wall_follow_clearance_min = node_param_.wall_follow_clearance_min;
  double wall_follow_clearance_max = node_param_.wall_follow_clearance_max;
  double min_speed = node_param_.min_speed;
  double slow_speed = node_param_.slow_speed;
  double max_speed = node_param_.max_speed;
  
  
  sensor_msgs::msg::LaserScan pub_laser;
  pub_laser.header.frame_id = "base_link";
  //pub_laser.header.stamp = msg.header.stamp;
  pub_laser.angle_increment = scan_angle_increment;
  pub_laser.angle_max = scan_angle_max;
  pub_laser.angle_min = scan_angle_min;
  pub_laser.range_max = scan_range_max;
  pub_laser.range_min = scan_range_min;
  pub_laser.scan_time = 0;
  pub_laser.ranges = scan_data;
  pub_grass_scan_->publish(pub_laser);
  */
  
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(image_, sensor_msgs::image_encodings::BGR8);
  cv::Mat img = cv_ptr->image; //cv_ptr->image;

  cv_bridge::CvImage pub_image_msg;
  pub_image_msg.header = image_->header;
  pub_image_msg.image = img;
  pub_image_msg.encoding = sensor_msgs::image_encodings::BGR8; //BGR8;
  pub_image_.publish(pub_image_msg.toImageMsg());
  
  cv_bridge::CvImage pub_test_image_msg;
  pub_test_image_msg.header = image_->header;
  pub_test_image_msg.image = img;
  pub_test_image_msg.encoding = sensor_msgs::image_encodings::BGR8; //BGR8;
  pub_test_image_.publish(pub_test_image_msg.toImageMsg());
  
 RCLCPP_INFO_STREAM(get_logger(), "x");
}

rcl_interfaces::msg::SetParametersResult SmartGapFollowNode::onSetParam(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;

  try {
    // Node Parameter
    {
      auto & p = node_param_;

      // Update params
      //update_param(params, "bev_width", p.bev_width);
      //update_param(params, "bev_height", p.bev_height);
      //update_param(params, "bev_origin", p.bev_origin);
      update_param(params, "px_dist", p.px_dist);
    }
  } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
    result.successful = false;
    result.reason = e.what();
    return result;
  }

  result.successful = true;
  result.reason = "success";
  return result;
}
}  // namespace radar_object_clustering

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(smart_gap_follow::SmartGapFollowNode)