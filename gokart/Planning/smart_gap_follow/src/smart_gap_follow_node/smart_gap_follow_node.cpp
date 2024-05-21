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
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <unordered_map>
#include <climits>

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
  node_param_.steering_speed = declare_parameter<double>("steering_speed");
  node_param_.range_thresh_min = declare_parameter<double>("range_thresh_min");
  node_param_.range_thresh_max = declare_parameter<double>("range_thresh_max");
  node_param_.min_gap_size = declare_parameter<double>("min_gap_size");
  node_param_.goal_angle = declare_parameter<double>("goal_angle");
  node_param_.wall_follow_clearance_min = declare_parameter<double>("wall_follow_clearance_min");
  node_param_.wall_follow_clearance_max = declare_parameter<double>("wall_follow_clearance_max");
  node_param_.min_speed = declare_parameter<double>("min_speed");
  node_param_.slow_speed = declare_parameter<double>("slow_speed");
  node_param_.max_speed = declare_parameter<double>("max_speed");

  // Subscriber
  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  sub_track_scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "~/input/track_scan", qos, std::bind(&SmartGapFollowNode::onTrackScan, this, std::placeholders::_1));

  sub_lidar_scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "~/input/lidar_scan", qos, std::bind(&SmartGapFollowNode::onLidarScan, this, std::placeholders::_1));

  sub_cam_image_ = create_subscription<sensor_msgs::msg::Image>(
    "~/input/cam_image", rclcpp::QoS{1},
    std::bind(&SmartGapFollowNode::onCamImage, this, std::placeholders::_1));

  sub_track_bev_ = create_subscription<sensor_msgs::msg::Image>(
    "~/input/track_bev", rclcpp::QoS{1},
    std::bind(&SmartGapFollowNode::onTrackBev, this, std::placeholders::_1));  

  // Publisher
  pub_drive_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("~/out/drive", 10);
  pub_planning_cam_ = image_transport::create_publisher(this, "~/out/planning_cam");
  pub_planning_bev_ = image_transport::create_publisher(this, "~/out/planning_bev");
}

void SmartGapFollowNode::findGap(const sensor_msgs::msg::LaserScan scan)
{
  const float min_gap_size = node_param_.min_gap_size;
  //const double range_thresh_min = node_param_.range_thresh_min;
  const float range_thresh_max = node_param_.range_thresh_max;
  range_thresh = range_thresh_max;
  const int scan_size = scan.ranges.size();
  const float angle_increment = scan.angle_increment;
  const float angle_min = scan.angle_min;
  const float angle_max = scan.angle_max;
  std::vector<float> scan_ranges = scan.ranges;
  std::vector<int> scan_ids(scan_size, 0);
  
  scan_ranges[0] = std::min(scan_ranges[0], 2 * min_gap_size);
  scan_ranges[scan_size - 1] = std::min(scan_ranges[scan_size - 1], 2 * min_gap_size);

  int new_id = 1;
  for(int i = 0; i < (int)scan_ranges.size(); i++){
    if(scan_ranges[i] > range_thresh)
        continue;

    if(scan_ids[i] == 0){
        scan_ids[i] = new_id;
        new_id++;
    }

    const float check_limit = min_gap_size < scan_ranges[i] ? 
          std::min(float(angle_min + i * angle_increment + asin(min_gap_size / scan_ranges[i])), angle_max) : angle_max;

    for(int j = i + 1; angle_min + j * angle_increment <= check_limit; j++){  
      if(scan_ranges[j] > range_thresh || scan_ids[i] == scan_ids[j])
        continue;
        
      const float dist = sqrt(pow(scan_ranges[i], 2) + pow(scan_ranges[j], 2) -  
                  2 * scan_ranges[i] * scan_ranges[j] * cos(abs(i - j) * angle_increment));
      //RCLCPP_INFO_STREAM(get_logger(), "dist:"<<dist);
      if(dist <= min_gap_size){
        if(scan_ids[j] == 0)
            scan_ids[j] = scan_ids[i];
        else{
          //cout<<"\nmerge:i:"<<i<<", j:"<<j;
          for(int k = 0; k <= j; k++){
            if(scan_ids[k] == scan_ids[i])
              scan_ids[k] = scan_ids[j]; //merge to the larger number cluster label
          }
        }
      }
    }
  }
  
  //reassign group numers in increasing order such that it no longer skip numbers after merging groups
  new_id = 1;
  int max_id = *max_element(scan_ids.begin(), scan_ids.end());
  for(int i = 1; i <= max_id; i++){
    bool id_found = false;
    for(int j = 0; j < (int)scan_ids.size(); j++){
      if(scan_ids[j] == i){
        scan_ids[j] = new_id;
        id_found = true;
      }
    }
    if(id_found)
      new_id++;
  }
  max_id = *max_element(scan_ids.begin(), scan_ids.end());
  
  //RCLCPP_INFO_STREAM(get_logger(), "num_group:"<<max_id);
  //for(int i = 0; i < (int)scan_ranges.size(); i++)
  //  RCLCPP_INFO_STREAM(get_logger(), i<<":["<<scan_ranges[i]<<","<<scan_ids[i]<<"]");
  
  std::unordered_map<int, std::pair<int, int>> group_map;
  std::unordered_map<int, float> group_min_dist_map;
  for(int i = 0; i < (int)scan_ranges.size(); i++){
    if(scan_ids[i] == 0)
      continue;
    if (group_map.find(scan_ids[i]) == group_map.end()){
      group_map[scan_ids[i]] = {i, i};
      group_min_dist_map[scan_ids[i]] = scan_ranges[i];
    }else{
      int curr_min_idx = group_map[scan_ids[i]].first;
      int curr_max_idx = group_map[scan_ids[i]].second;
      group_map[scan_ids[i]] = {std::min(curr_min_idx, i), std::max(curr_max_idx, i)};
      group_min_dist_map[scan_ids[i]] = std::min(group_min_dist_map[scan_ids[i]], scan_ranges[i]);
    }
  }
  
  /*
  cout<<"group_min_dist_map";
  for(auto i: group_min_dist_map){
    cout<<"\nid:"<<i.first<<" min_dist:"<< i.second;
  }
  for(auto i: group_map)
    cout<<"\ngroup id before:"<<i.first<<"["<<i.second.first<<","<<i.second.second<<"]";
    */
  
  std::vector<int> ids_to_remove;
  //remove the rear obstacle groups when two groups overlap each other
  for(int i = 1; i <= max_id; i++){
      for(int j = 1; j <= max_id; j++){
          if(i != j && std::max(group_map[i].first, group_map[j].first)
             <= std::min(group_map[i].second, group_map[j].second)){
              //cout<<"overlap!!!"<<i<<","<<j<<"\n";
              if(group_min_dist_map[i] < group_min_dist_map[j] && group_min_dist_map[i] != 0){
                  ids_to_remove.push_back(j);
                  group_map.erase(j);
                  //cout<<"erase id:"<<j;
              }
              if(group_min_dist_map[j] < group_min_dist_map[i] && group_min_dist_map[j] != 0){
                  ids_to_remove.push_back(i);
                  group_map.erase(i);
                  //cout<<"erase id:"<<i;
              }
          }
      }
  }
  //for(auto i:ids_to_remove)
  //  RCLCPP_INFO_STREAM(get_logger(), "remove:"<<i<<",");
  //for(auto i: group_map)
    //cout<<"\ngroup id after:"<<i.first<<"["<<i.second.first<<","<<i.second.second<<"]";

  std::vector<std::pair<int, int>> groups;
  for(auto it:group_map)
    if(find(ids_to_remove.begin(), ids_to_remove.end(), it.first) == ids_to_remove.end())
      groups.push_back(it.second);
  reverse(groups.begin(), groups.end());
  
  deadend = (groups[0].first == 0 && groups[0].second == 180);

  //cout<<"\ngroup:";
  //for(auto it: groups)
  //  cout<<"["<<it.first<<","<<it.second<<"]";
  
  gap_indices.clear();
  for(int i = 0; i < (int)groups.size()-1; i++)
    gap_indices.push_back({groups[i].second, groups[i+1].first});
  
  if(groups.empty())
    gap_indices.push_back({60, 120});

  //RCLCPP_INFO_STREAM(get_logger(), "gaps-----------------------------------");
  //for(auto it:gap_indices)
  //  RCLCPP_INFO_STREAM(get_logger(), "["<<it.first<<","<<it.second<<"]");
  
} 

void SmartGapFollowNode::findTargetGap(const sensor_msgs::msg::LaserScan scan)
{
  const float angle_increment = scan.angle_increment;
  const float angle_min = scan.angle_min;
  //const float angle_max = scan.angle_max;
  const float goal_angle = node_param_.goal_angle;
  float min_angle_diff = INT_MAX;
  for(auto it:gap_indices){
    float angle_diff_first = abs(angle_min + it.first * angle_increment - goal_angle);
    float angle_diff_second = abs(angle_min + it.second * angle_increment - goal_angle);
    if(std::min(angle_diff_first, angle_diff_second) < min_angle_diff){
      min_angle_diff = std::min(angle_diff_first, angle_diff_second);
      target_gap_indices = it;
    }
  }
}

void SmartGapFollowNode::calcMotionCmd(const sensor_msgs::msg::LaserScan scan)
{
  const float min_speed = node_param_.min_speed;
  //const float slow_speed = node_param_.slow_speed;
  const float max_speed = node_param_.max_speed;
  const float angle_increment = scan.angle_increment;
  const float angle_min = scan.angle_min;
  const float wall_follow_clearance_min = node_param_.wall_follow_clearance_min;
  const float car_width = node_param_.car_width;

  float gap_angle_min = angle_min + target_gap_indices.first * angle_increment;
  float gap_angle_max = angle_min + target_gap_indices.second * angle_increment;
  //float dist_to_gap = std::min(scan.ranges[target_gap_indices.first], 
  //  scan.ranges[target_gap_indices.second]);
  float gap_angle_min_x = scan.ranges[target_gap_indices.first] * sin(gap_angle_min);
  float gap_angle_min_y = scan.ranges[target_gap_indices.first] * cos(gap_angle_min);
  float gap_angle_max_x = scan.ranges[target_gap_indices.second] * sin(gap_angle_max);
  float gap_angle_max_y = scan.ranges[target_gap_indices.second] * cos(gap_angle_max);
  float gap_size = sqrt(pow(gap_angle_min_x - gap_angle_max_x, 2) + pow(gap_angle_min_y - gap_angle_max_y,2));
  //float gap_angle = atan((gap_angle_min_y - gap_angle_max_y) / (gap_angle_min_x - gap_angle_max_x));
  
  //RCLCPP_INFO_STREAM(get_logger(), "dist_to_gap: "<<dist_to_gap<< " m");
  //RCLCPP_INFO_STREAM(get_logger(), "gap_size: "<<gap_size<<" m");
  //RCLCPP_INFO_STREAM(get_logger(), "gap angle:"<<gap_angle * 180 / M_PI);
  //RCLCPP_INFO_STREAM(get_logger(), "gap_angle_min_x:"<<gap_angle_min_x <<" gap_angle_min_y:"<<gap_angle_min_y);
  //RCLCPP_INFO_STREAM(get_logger(), "gap_angle_max_x:"<<gap_angle_max_x <<" gap_angle_max_y:"<<gap_angle_max_y);

  float steer_limit_min = gap_angle_min + 
    atan((car_width / 2 + wall_follow_clearance_min) / scan.ranges[target_gap_indices.first]);
  float steer_limit_max = gap_angle_max - 
    atan((car_width / 2 + wall_follow_clearance_min) / scan.ranges[target_gap_indices.second]);
  

  /*
  steer_limit_min = std::min(float(40.0 * M_PI / 180), steer_limit_min);
  steer_limit_max = std::min(float(40.0 * M_PI / 180), steer_limit_max);
  steer_limit_min = std::max(float(-40.0 * M_PI / 180), steer_limit_min);
  steer_limit_max = std::max(float(-40.0 * M_PI / 180), steer_limit_max);
  */
  float steer_angle;
  if(abs(steer_limit_min) * 180 / M_PI > 80 || abs(steer_limit_max) * 180 / M_PI > 80){
    if(abs(steer_limit_min) * 180 / M_PI > 80)
      steer_angle = steer_limit_max * 1.6;
    else
      steer_angle = steer_limit_min * 1.6;
  }else if(steer_limit_max > steer_limit_min){
    steer_angle = (steer_limit_min + steer_limit_max) / 2;
  }else{
    if(scan.ranges[target_gap_indices.first] > scan.ranges[target_gap_indices.second])
      steer_angle = steer_limit_max;
    else
      steer_angle = steer_limit_min;
  }
  if(abs(steer_angle) < 10)
    steer_angle *= 1;

  if(abs(steer_angle) >= 15)
    steer_angle *= 1.6;

  float target_speed = max_speed * (1 - abs(steer_angle * 180 / M_PI) / 30);// * pow(std::min((float)1.0, dist_to_gap / 12), 1.5);
  if(gap_size < 4.0)
    target_speed = min_speed;
  target_speed = max_speed;
  RCLCPP_INFO_STREAM(get_logger(), "target_speed: "<<target_speed << " m/s");
  //RCLCPP_INFO_STREAM(get_logger(), "steer_limits:["<<steer_limit_min * 180 / M_PI <<", "<<
  //  steer_limit_max * 180 / M_PI <<"]");
  RCLCPP_INFO_STREAM(get_logger(), "steer_angle: "<<steer_angle * 180 / M_PI << " deg");

  ackermann_msgs::msg::AckermannDriveStamped ackermann_msg;
  ackermann_msg.drive.speed = target_speed;
  ackermann_msg.drive.steering_angle = steer_angle;
  ackermann_msg.drive.steering_angle_velocity = 1.0;
  ackermann_msg.drive.acceleration = 2.0;
  pub_drive_->publish(ackermann_msg);  
}

sensor_msgs::msg::LaserScan SmartGapFollowNode::merge_scan(const sensor_msgs::msg::LaserScan scan_1,
  const sensor_msgs::msg::LaserScan scan_2)
{
  if(scan_1.ranges.size() != scan_2.ranges.size()){
    RCLCPP_INFO_STREAM(get_logger(), "merge scan error");
    return scan_1;
  }
  sensor_msgs::msg::LaserScan merged_scan;
  merged_scan.angle_increment = scan_1.angle_increment;
  merged_scan.angle_min = scan_1.angle_min;
  merged_scan.angle_max = scan_1.angle_max;
  merged_scan.range_min = scan_1.range_min;
  merged_scan.range_max = scan_1.range_max;
  merged_scan.ranges = scan_1.ranges;
  for(int i = 0; i < (int)scan_1.ranges.size(); i++)
    merged_scan.ranges[i] = scan_1.ranges[i] < scan_2.ranges[i] ? scan_1.ranges[i] : scan_2.ranges[i];
  return merged_scan;
}

void SmartGapFollowNode::onTrackScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr track_scan_)
{
  track_scan.angle_increment = track_scan_->angle_increment;
  track_scan.angle_min = track_scan_->angle_min;
  track_scan.angle_max = track_scan_->angle_max;
  track_scan.range_min = track_scan_->range_min;
  track_scan.range_max = track_scan_->range_max;
  track_scan.ranges = track_scan_->ranges;
  merged_scan = merge_scan(track_scan, lidar_scan);
  findGap(merged_scan);
  findTargetGap(merged_scan);
  calcMotionCmd(merged_scan);
  //RCLCPP_INFO_STREAM(get_logger(), "gap");
}

void SmartGapFollowNode::onLidarScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr lidar_scan_)
{
  lidar_scan.angle_increment = lidar_scan_->angle_increment;
  lidar_scan.angle_min = lidar_scan_->angle_min;
  lidar_scan.angle_max = lidar_scan_->angle_max;
  lidar_scan.range_min = lidar_scan_->range_min;
  lidar_scan.range_max = lidar_scan_->range_max;
  lidar_scan.ranges = lidar_scan_->ranges;
}

void SmartGapFollowNode::onCamImage(const sensor_msgs::msg::Image::ConstSharedPtr cam_image_)
{
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(cam_image_, sensor_msgs::image_encodings::BGR8);
  cv::Mat cam_image = cv_ptr->image; 

  cv_bridge::CvImage pub_planning_cam_msg;
  pub_planning_cam_msg.header = cam_image_->header;
  pub_planning_cam_msg.image = cam_image;
  pub_planning_cam_msg.encoding = sensor_msgs::image_encodings::BGR8; //BGR8;
  pub_planning_cam_.publish(pub_planning_cam_msg.toImageMsg());
  RCLCPP_INFO_STREAM(get_logger(), " ");
}

void SmartGapFollowNode::onTrackBev(const sensor_msgs::msg::Image::ConstSharedPtr track_bev_)
{
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(track_bev_, sensor_msgs::image_encodings::BGR8);
  cv::Mat track_bev = cv_ptr->image; 
  cv_bridge::CvImage pub_planning_bev_msg;
  pub_planning_bev_msg.header = track_bev_->header;
  pub_planning_bev_msg.image = track_bev;
  pub_planning_bev_msg.encoding = sensor_msgs::image_encodings::MONO8; //BGR8;
  pub_planning_bev_.publish(pub_planning_bev_msg.toImageMsg());
  RCLCPP_INFO_STREAM(get_logger(), " ");
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
}  // namespace 

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(smart_gap_follow::SmartGapFollowNode)