//  Copyright 2022 Tier IV, Inc. All rights reserved.
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.

#include "smart_gap_follow_nodes/smart_gap_follow.hpp"

namespace smart_gap_follow
{

SmartGapFollow::SmartGapFollow(const rclcpp::NodeOptions & options)
: Node("smart_gap_follow", options)
{ 
  drive_pub_ = create_publisher<AckermannDriveStamped>("/drive", 1);
  target_gap_pub_ = create_publisher<Marker>("/target_gap", 1);
  vis_goal_pub_ = create_publisher<visualization_msgs::msg::Marker>( "visualization_goal_topic", 10 );
  vis_path_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>( "visualization_path_topic", 10);
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "/pf/pose/odom", 10, [this](const Odometry::SharedPtr msg) { odometry_ = msg; });
  scan_sub_ = create_subscription<LaserScan>(
    "/scan", 1, [this](const LaserScan::SharedPtr msg) { scan_ = msg; });
  
  mode_ = declare_parameter<string>("mode", "gap_follow");
  wall_clearance_ = declare_parameter<double>("wall_clearance_", 0.5);
  group_radius_ = declare_parameter<double>("group_radius", 0.7);
  max_range_ = declare_parameter<double>("max_range", 5.0);
  min_range_ = declare_parameter<double>("min_range", 2.7);
  target_dir_ = declare_parameter<double>("target_dir", 90.0);
  boost_ = declare_parameter<bool>("boost", false);
  boost_speed_ = declare_parameter<double>("boost_speed", 6.0);
  max_speed_ = declare_parameter<double>("max_speed", 4.6);
  inter_speed_ = declare_parameter<double>("inter_speed", 2.6);
  min_speed_ = declare_parameter<double>("min_speed", 2.2);
  slow_down_steer_angle_ = declare_parameter<double>("slow_down_steer_angle", 0.5);
  slow_down_gap_dist_ = declare_parameter<double>("slow_down_gap_dist", 2.8);

  range_thres = max_range_;
  //parseCSV();

  using namespace std::literals::chrono_literals;
  timer_ = rclcpp::create_timer(
    this, get_clock(), 10ms, std::bind(&SmartGapFollow::onTimer, this));
}

void SmartGapFollow::preproc_scan(){
  //double data_res = scan_->angle_increment * 180 / M_PI;
  int crop =  45 * 4 - 2;
  scan_ranges = {scan_->ranges.begin() + crop, scan_->ranges.end() - crop};
  for(int i = 0; i <= 180; i++){
    scan_ranges[i] = *min_element(scan_ranges.begin() + int(4 * i), 
                                  scan_ranges.begin() + int(4 * i + 3));
  }
  scan_ranges.erase(scan_ranges.begin() + 181, scan_ranges.end());
  /*
  cout<<"scan:";
  for(auto range: scan_ranges)
    cout<<range<<",";
  */
}

void SmartGapFollow::update_range_thres(){
  range_thres = deadend ? min_range_ : 
                min_range_ + (max_range_ - min_range_) * cos(gap_steer * M_PI/180);
  cout<<"range_thres:"<<range_thres;
}

void SmartGapFollow::find_gaps(){
  vector<int> scan_ids(181, 0);
  int new_id = 1;
  for(int i = 0; i < (int)scan_ranges.size(); i++){
    if(scan_ranges[i] > range_thres)
        continue;

    if(scan_ids[i] == 0){
        scan_ids[i] = new_id;
        new_id++;
    }

    double check_limit = group_radius_ < scan_ranges[i] ? 
          min((int)ceil(i + asin(group_radius_ / scan_ranges[i]) * 180 / M_PI), 180) : 180;

    for(int j = i+1; j <= check_limit; j++){  
      if(scan_ranges[j] > range_thres || scan_ids[i] == scan_ids[j])
        continue;
        
      double dist = sqrt(pow(scan_ranges[i], 2) + pow(scan_ranges[j], 2) -  
                  2 * scan_ranges[i] * scan_ranges[j] * cos(abs(i - j) * M_PI / 180));

      if(dist <= group_radius_){
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
  /*
  cout<<"\nrange,id:";
  for(int i = 0; i < (int)scan_ids.size(); i++)
    cout<<i<<"["<<scan_ranges[i]<<","<<scan_ids[i]<<"],";
  */
  unordered_map<int, pair<int, int>> group_map;
  unordered_map<int, double> group_min_dist_map;
  for(int i = 0; i < (int)scan_ranges.size(); i++){
    if(scan_ids[i] == 0)
      continue;
    if (group_map.find(scan_ids[i]) == group_map.end()){
      group_map[scan_ids[i]] = {i, i};
      group_min_dist_map[scan_ids[i]] = INT_MAX;
    }else{
      int curr_min_angle = group_map[scan_ids[i]].first;
      int curr_max_angle = group_map[scan_ids[i]].second;
      group_map[scan_ids[i]] = {min(curr_min_angle, i), max(curr_max_angle, i)};
      group_min_dist_map[scan_ids[i]] = min(group_min_dist_map[scan_ids[i]], scan_ranges[i]);
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
  vector<int> ids_to_remove;
  //remove the rear obstacle groups when two groups overlap each other
  for(int i = 1; i <= max_id; i++){
      for(int j = 1; j <= max_id; j++){
          if(i != j && max(group_map[i].first, group_map[j].first)
             <= min(group_map[i].second, group_map[j].second)){
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
  //for(auto i: group_map)
    //cout<<"\ngroup id after:"<<i.first<<"["<<i.second.first<<","<<i.second.second<<"]";

  vector<pair<int, int>> groups;
  for(auto it:group_map)
    if(find(ids_to_remove.begin(), ids_to_remove.end(), it.first) == ids_to_remove.end() &&
      (it.second.first != 0 || it.second.second != 0))
      groups.push_back(it.second);
  reverse(groups.begin(), groups.end());
  
  deadend = (groups[0].first == 0 && groups[0].second == 180);

  cout<<"\ngroup:";
  for(auto it: groups)
    cout<<"["<<it.first<<","<<it.second<<"]";
  
  gaps.clear();
  for(int i = 0; i < (int)groups.size()-1; i++)
    gaps.push_back({groups[i].second, groups[i+1].first});
  
  if(groups.empty())
    gaps.push_back({60, 120});

  cout<<"\ngaps:";
  for(auto it:gaps)
    cout<<"["<<it.first<<","<<it.second<<"]";
  
}

void SmartGapFollow::find_target_gaps(){
  //double min_angle_from_mid = INT_MAX;
  double min_gap_length = INT_MAX;
  //double min_angle = INT_MAX;
  double x1, x2, y1, y2;
  for(auto gap:gaps){
    /*
    double angle_offset = min(abs(gap.first - target_dir_), abs(gap.second - target_dir_));
    if(angle_offset < min_angle_from_mid){
      target_gap = gap;
      min_angle_from_mid = angle_offset;
    }
    */
    /*
    if(abs(gap.second - gap.first) < min_angle){
      target_gap = gap;
      min_angle = abs(gap.second - gap.first);
    }*/
    x1 = scan_ranges[gap.first] * cos(gap.first * M_PI/180);
    y1 = scan_ranges[gap.first] * sin(gap.first * M_PI/180);
    x2 = scan_ranges[gap.second] * cos(gap.second * M_PI/180);
    y2 = scan_ranges[gap.second] * sin(gap.second * M_PI/180);
    gap_length = sqrt(pow((x1 - x2), 2) + pow((y1 - y2), 2));
    if(gap_length < min_gap_length){
      target_gap = gap;
      min_gap_length = gap_length;
      gap_angle = scan_ranges[gap.first] < scan_ranges[gap.second] ?
              atan2(y2 - y1, x2 - x1) : atan2(y1 - y2, x1 - x2);
      gap_angle -= M_PI/2;
    }
  }
  cout<<"\ntarget_gap:["<<target_gap.first<<","<<target_gap.second<<","<<min_gap_length<<"]";
  
  if(!gaps.empty())
    pubGapMarker(target_gap, {0, 1, 0});
}

void SmartGapFollow::pubGapMarker(pair<double, double> target_gap, vector<double> color){
  Marker marker;
  marker.type = marker.POINTS;
  marker.header.frame_id = "laser";
  Point right_edge;
  right_edge.x = scan_ranges[target_gap.first] * sin(target_gap.first * M_PI / 180);
  right_edge.y = -scan_ranges[target_gap.first] * cos(target_gap.first * M_PI / 180);
  right_edge.z = 0;
  Point left_edge;
  left_edge.x = scan_ranges[target_gap.second] * sin(target_gap.second * M_PI / 180);
  left_edge.y = -scan_ranges[target_gap.second] * cos(target_gap.second * M_PI / 180);
  left_edge.z = 0;
  //marker.pose = pose;
  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;
  marker.color.r = color[0];
  marker.color.g = color[1];
  marker.color.b = color[2];
  marker.color.a = 1.0;
  marker.lifetime = rclcpp::Duration::from_nanoseconds(0.03 * 1e9);

  /*
  for(auto point: data){
    point.x = pt[i] * cos(i * M_PI / 180);
    point.y = pt[i] * sin(i * M_PI / 180);
    point.z = 0.0;
    marker.points.push_back(point);
  }
  */
  marker.points = {right_edge, left_edge};
  target_gap_pub_->publish(marker);
}

void SmartGapFollow::onTimer()
{
  /*
  if (!checkData()) {
     RCLCPP_INFO(get_logger(), "data not ready!!");
    return;
  }
  */
  //createTrajectoryMarker();
  auto start_time = high_resolution_clock::now();
  preproc_scan();
  //createRvizMarker(scan_ranges, {255, 0, 0});
  find_gaps();
  find_target_gaps();
  
  /*
  AckermannControlCommand cmd;
  cmd.stamp = cmd.lateral.stamp = cmd.longitudinal.stamp = get_clock()->now();
  cmd.lateral.steering_tire_angle = static_cast<double>(calcSteerCmd());

  float max_speed = boost_ && abs(gap_steer) < 2 ? max_speed_ + 1 : max_speed_;

  double speed_out = min_speed_ + (max_speed - min_speed_) * cos((abs(3 * gap_steer)));
  if((abs(gap_angle) > M_PI/3 && cross_over) || dist_to_gap < slow_down_gap_dist_)
    speed_out = min(inter_speed_, speed_out);
  
  if(deadend && range_thres == min_range_)
    speed_out = 0;

  cmd.longitudinal.speed = static_cast<double>(speed_out);
  cmd.longitudinal.acceleration = static_cast<double>(calcAccCmd());

  ackermann_msgs::msg::AckermannDriveStamped ackermann_msg;
  ackermann_msg.drive.speed = cmd.longitudinal.speed;
  ackermann_msg.drive.steering_angle = cmd.lateral.steering_tire_angle;
  drive_pub_->publish(ackermann_msg);
  
  cout << " velocity: " << ackermann_msg.drive.speed << "m/s"<< endl;
  */
  auto duration = duration_cast<microseconds>(high_resolution_clock::now() - start_time);
  double duration_cnt = duration.count();
  cout << "\ncomputation time: "<< duration_cnt/1000<<" ms \n\n"; 
  
}

double SmartGapFollow::calcSteerCmd()
{
  double steer_limit_right;
  if(scan_ranges[target_gap.first] > wall_clearance_ && target_gap.first > 45)
    steer_limit_right = target_gap.first * M_PI/180 + asin(wall_clearance_ / scan_ranges[target_gap.first]);
  else
    steer_limit_right = target_gap.first * M_PI/180 + M_PI/10;
  
  double steer_limit_left;
  if(scan_ranges[target_gap.second] > wall_clearance_ && target_gap.second < 135)
    steer_limit_left = target_gap.second * M_PI/180 - asin(wall_clearance_ / scan_ranges[target_gap.second]);
  else
    steer_limit_left = target_gap.second * M_PI/180 - M_PI/10;

  //cout<<"\nsteer_limit_left:"<<steer_limit_left;
  //cout<<"\nsteer_limit_right:"<<steer_limit_right;

  steer_limit_right -= M_PI/2;
  steer_limit_left -= M_PI/2;

  cross_over = steer_limit_right > steer_limit_left;

  if(!cross_over){
    if(steer_limit_right - steer_limit_left > 60){
      gap_steer = steer_limit_left;
    }else{
      gap_steer = (steer_limit_right + steer_limit_left) / 2;
    }
    dist_to_gap = 100.0;

  }else{
    gap_steer = scan_ranges[target_gap.first] < scan_ranges[target_gap.second] ?
                steer_limit_right : steer_limit_left;
    
    dist_to_gap = scan_ranges[target_gap.first] < scan_ranges[target_gap.second] ?
                  scan_ranges[target_gap.first] : scan_ranges[target_gap.second];
  }

  if(dist_to_gap > 1.2 || abs(gap_steer) < 0.6)
    gap_steer *= 0.55;

  constexpr auto steer_lim = 1.0;
  const auto steer_out = std::clamp(gap_steer, -steer_lim, steer_lim);
  gap_steer = abs(gap_steer) > M_PI/6 ? M_PI/6: gap_steer;

  cout<<"\nsteer angle:"<< steer_out * 180/M_PI<<", "<<steer_out;
  return steer_out;
}

double SmartGapFollow::calcAccCmd()
{
  /*
  const auto traj_vel = static_cast<double>(3);
  const auto ego_vel = odometry_->twist.twist.linear.x;
  const auto target_vel = traj_vel;//use_external_target_vel_ ? external_target_vel_ : traj_vel;
  const auto vel_err = ego_vel - target_vel;

  // P feedback
  constexpr auto kp = 0.5;
  constexpr auto acc_lim = 2.0;

  const auto acc = std::clamp(-kp * vel_err, -acc_lim, acc_lim);
  RCLCPP_DEBUG(get_logger(), "vel_err = %f, acc = %f", vel_err, acc);
  */
  const auto acc = 1.5;
  return acc;
}

bool SmartGapFollow::checkData() { return scan_ && odometry_; }

  // namespace f1tenth_advanced_gap_follower

void SmartGapFollow::visualize_path_points()
{
    auto marker_array = visualization_msgs::msg::MarkerArray();
    auto marker = visualization_msgs::msg::Marker();
    marker.header.frame_id = "map";
    marker.header.stamp = rclcpp::Clock().now();
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.15;
    marker.scale.y = 0.15;
    marker.scale.z = 0.15;
    marker.color.a = 1.0; 
    marker.color.g = 1.0;

    for(int i=0; i<(int)waypoints.size(); ++i)
    {
        marker.pose.position.x = waypoints[i][0];
        marker.pose.position.y = waypoints[i][1];
        marker.id = i;
        marker_array.markers.push_back(marker);
    }

    vis_path_pub_->publish(marker_array);
}


}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(smart_gap_follow::SmartGapFollow)
