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

#include "smart_gap_follow_node/smart_gap_follow_node.hpp"
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
  node_param_.bev_width = declare_parameter<int>("bev_width");
  node_param_.bev_height = declare_parameter<int>("bev_height");
  node_param_.bev_origin = declare_parameter<std::vector<int64_t>>("bev_origin");
  node_param_.px_dist = declare_parameter<double>("px_dist");
  node_param_.grass_b_weight = declare_parameter<double>("grass_b_weight");
  node_param_.grass_g_weight = declare_parameter<double>("grass_g_weight");
  node_param_.cone_b_weight = declare_parameter<double>("cone_b_weight");
  node_param_.cone_r_weight = declare_parameter<double>("cone_r_weight");
  node_param_.grass_thres = declare_parameter<int>("grass_thres");
  node_param_.cone_thres = declare_parameter<int>("cone_thres");
  node_param_.cone_base_ratio = declare_parameter<double>("cone_base_ratio");
  node_param_.img_bot_left = declare_parameter<std::vector<int64_t>>("img_bot_left");
  node_param_.img_top_left = declare_parameter<std::vector<int64_t>>("img_top_left");
  node_param_.img_top_right = declare_parameter<std::vector<int64_t>>("img_top_right");
  node_param_.img_bot_right = declare_parameter<std::vector<int64_t>>("img_bot_right");
  node_param_.world_bot_left = declare_parameter<std::vector<double>>("world_bot_left");
  node_param_.world_top_left = declare_parameter<std::vector<double>>("world_top_left");
  node_param_.world_top_right = declare_parameter<std::vector<double>>("world_top_right");
  node_param_.world_bot_right = declare_parameter<std::vector<double>>("world_bot_right");
  node_param_.scan_angle_increment = declare_parameter<double>("scan_angle_increment");
  node_param_.scan_angle_min = declare_parameter<double>("scan_angle_min");
  node_param_.scan_angle_max = declare_parameter<double>("scan_angle_max");
  node_param_.scan_range_min = declare_parameter<double>("scan_range_min");
  node_param_.scan_range_max = declare_parameter<double>("scan_range_max");
  node_param_.ray_dist_step = declare_parameter<int>("ray_dist_step");

  // Subscriber
  sub_image_ = create_subscription<sensor_msgs::msg::Image>(
    "~/input/image", rclcpp::QoS{1},
    std::bind(&SmartGapFollowNode::onImage, this, std::placeholders::_1));

  // Publisher
  //pub_image_ = create_publisher<sensor_msgs::msg::Image>("~/output/image", 1);
  pub_image_ = image_transport::create_publisher(this, "~/out/image");
  pub_test_image_ = image_transport::create_publisher(this, "~/test/image");
  pub_grass_scan_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
    "~/out/grass_scan", rclcpp::SensorDataQoS());
}

void SmartGapFollowNode::onImage(const sensor_msgs::msg::Image::ConstSharedPtr image_)
{
  
  cv::Size bev_size = cv::Size(node_param_.bev_width, node_param_.bev_height);
  std::vector<int64_t> bev_origin = node_param_.bev_origin;
  double px_dist = node_param_.px_dist;
  double grass_b_weight = node_param_.grass_b_weight;
  double grass_g_weight = node_param_.grass_g_weight;
  double cone_b_weight = node_param_.cone_b_weight;
  double cone_r_weight = node_param_.cone_r_weight;
  int grass_thres = node_param_.grass_thres;
  int cone_thres = node_param_.cone_thres;
  double cone_base_ratio = node_param_.cone_base_ratio;
  std::vector<int64_t> img_bot_left = node_param_.img_bot_left;
  std::vector<int64_t> img_top_left = node_param_.img_top_left;
  std::vector<int64_t> img_top_right = node_param_.img_top_right;
  std::vector<int64_t> img_bot_right = node_param_.img_bot_right;
  std::vector<double> world_bot_left = node_param_.world_bot_left;
  std::vector<double> world_top_left = node_param_.world_top_left;
  std::vector<double> world_top_right = node_param_.world_top_right;
  std::vector<double> world_bot_right = node_param_.world_bot_right;
  
  double scan_angle_increment = node_param_.scan_angle_increment;
  double scan_angle_min = node_param_.scan_angle_min;
  double scan_angle_max = node_param_.scan_angle_max;
  double scan_range_min = node_param_.scan_range_min;
  double scan_range_max = node_param_.scan_range_max;
  int ray_dist_step = node_param_.ray_dist_step;
  
  cv::Point2f img_coords[4];
	cv::Point2f bev_coords[4];

  img_coords[0] = cv::Point2f(img_bot_left[0], img_bot_left[1]);
  img_coords[1] = cv::Point2f(img_top_left[0], img_top_left[1]);
  img_coords[2] = cv::Point2f(img_top_right[0], img_top_right[1]);
  img_coords[3] = cv::Point2f(img_bot_right[0], img_bot_right[1]);

  bev_coords[0] = cv::Point2f(bev_origin[0] + world_bot_left[0] / px_dist, 
                              bev_origin[1] - world_bot_left[1] / px_dist);
  bev_coords[1] = cv::Point2f(bev_origin[0] + world_top_left[0] / px_dist, 
                              bev_origin[1] - world_top_left[1] / px_dist);
  bev_coords[2] = cv::Point2f(bev_origin[0] + world_top_right[0] / px_dist, 
                              bev_origin[1] - world_top_right[1] / px_dist);
  bev_coords[3] = cv::Point2f(bev_origin[0] + world_bot_right[0] / px_dist, 
                              bev_origin[1] - world_bot_right[1] / px_dist);

  //RCLCPP_INFO_STREAM(get_logger(), bev_coords[1].y);
  cv::Mat mat = cv::getPerspectiveTransform(img_coords, bev_coords);

  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(image_, sensor_msgs::image_encodings::BGR8);
  cv::Mat img = cv_ptr->image; //cv_ptr->image;

  //cv::Mat img2 = cv::imread("/home/autoware/Pictures/Screenshots/8.png", cv::IMREAD_COLOR);
  cv::blur(img, img, cv::Size(10, 10));

  cv::Mat bgr[3];
  cv::split(img, bgr);

  cv::Mat b = bgr[0];
  cv::Mat g = bgr[1];
  cv::Mat r = bgr[2];

  cv::Mat grass_gray, grass_mask;
  cv::absdiff(grass_b_weight * b, grass_g_weight * g, grass_gray);
  cv::threshold(grass_gray, grass_mask, grass_thres, 255, cv::THRESH_BINARY);

  cv::Mat cone_gray, cone_mask;
  cv::absdiff(cone_b_weight * b, cone_r_weight * r, cone_gray);
  cv::threshold(cone_gray, cone_mask, cone_thres, 255, cv::THRESH_BINARY);
  std::vector<std::vector<cv::Point>> cone_contours, cone_base_contours;
  std::vector<cv::Vec4i> hierarchy;
  findContours(cone_mask, cone_contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

  cv::Mat kernel = getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
  dilate(cone_mask, cone_mask, kernel, cv::Point(-1, -1), 1);

  //cone_base mask
  for(auto cone_contour:cone_contours){
    cv::Rect rect = cv::boundingRect(cone_contour);
    std::vector<cv::Point> cone_base_contour;
    for(auto point:cone_contour){
      if(point.y > rect.y + rect.height * (1 - cone_base_ratio))
        cone_base_contour.push_back(point);
    }
    if(cone_base_contour.size() > 0)
      cone_base_contours.push_back(cone_base_contour);
  }

  cv::Mat cone_base_mask = cv::Mat::zeros(cone_mask.size(), cone_mask.type());
  drawContours(cone_base_mask, cone_base_contours, -1, 255, -1);

  cv::subtract(grass_mask, cone_mask, grass_mask);
  cv::add(grass_mask, cone_base_mask, grass_mask);

  //fill hole and remove noise
  cv::morphologyEx(grass_mask, grass_mask, cv::MORPH_OPEN, kernel);
  cv::morphologyEx(grass_mask, grass_mask, cv::MORPH_CLOSE, kernel);

  //bev projection
  cv::Mat bev, bev_grass;
  cv::warpPerspective(img, bev, mat, bev_size, cv::INTER_LINEAR);
  cv::warpPerspective(grass_mask, bev_grass, mat, bev_size, cv::INTER_LINEAR);

  //bev to lidar_scan
  const int num_data = int((scan_angle_max - scan_angle_min) / scan_angle_increment) + 1;
  std::vector<float> scan_data(num_data, scan_range_max);
  for(int i = 0; i < num_data; i++){
    int ray_dist_px = int(scan_range_min * px_dist);
    float ray_angle = 90 * M_PI / 180 - scan_angle_max + i * scan_angle_increment;
    while(ray_dist_px * px_dist <= scan_range_max){
      int x = int(bev_origin[0] + ray_dist_px * cos(ray_angle));
      int y = int(bev_origin[1] - ray_dist_px * sin(ray_angle));
      
      if(x >= bev_size.width || x < 0 || y >= bev_size.height || y < 0)
        break;
      if((int) bev_grass.at<uchar>(y,x) > 100){
        scan_data[i] = ray_dist_px * px_dist;
        break;
      }
      ray_dist_px += ray_dist_step;
    }
  }

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
  
  cv_bridge::CvImage pub_image_msg;
  pub_image_msg.header = image_->header;
  pub_image_msg.image = bev;
  pub_image_msg.encoding = sensor_msgs::image_encodings::BGR8; //BGR8;
  pub_image_.publish(pub_image_msg.toImageMsg());

  cv_bridge::CvImage pub_test_image_msg;
  pub_test_image_msg.header = image_->header;
  pub_test_image_msg.image = bev_grass;
  pub_test_image_msg.encoding = sensor_msgs::image_encodings::MONO8; //BGR8;
  pub_test_image_.publish(pub_test_image_msg.toImageMsg());
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
      update_param(params, "bev_width", p.bev_width);
      update_param(params, "bev_height", p.bev_height);
      update_param(params, "bev_origin", p.bev_origin);
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