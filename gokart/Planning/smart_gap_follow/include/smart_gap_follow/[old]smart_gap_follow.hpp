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


#include <rclcpp/rclcpp.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include "visualization_msgs/msg/marker_array.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <math.h>
#include <fstream>
#include <unordered_map>
#include <memory>
#include <vector>
#include <limits.h>
#include <chrono>
#include <algorithm>
#include <string>
//#include <Eigen/Geometry>

using namespace std;
using namespace std::chrono;

namespace smart_gap_follow
{
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Twist;
using nav_msgs::msg::Odometry;
using ackermann_msgs::msg::AckermannDriveStamped;
using visualization_msgs::msg::Marker;
using geometry_msgs::msg::Vector3;
using std_msgs::msg::Header;
using geometry_msgs::msg::Point;
using sensor_msgs::msg::LaserScan;
using std::vector;
using std::pair;

class SmartGapFollow : public rclcpp::Node
{
public:
  explicit SmartGapFollow(const rclcpp::NodeOptions & options);
  ~SmartGapFollow() = default;

private:
  rclcpp::Subscription<LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::ConstSharedPtr odom_sub_;
  rclcpp::Publisher<AckermannDriveStamped>::SharedPtr drive_pub_;
  rclcpp::Publisher<Marker>::SharedPtr target_gap_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vis_goal_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vis_path_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  Odometry::SharedPtr odometry_;
  LaserScan::SharedPtr scan_;

  vector<double> scan_ranges;
  double range_thres;
  vector<pair<int, int>> gaps;
  pair<int, int> target_gap;
  double gap_length;
  double gap_angle;
  bool   deadend;
  bool   cross_over;
  double dist_to_gap;
  double gap_steer;
  vector<vector<double>> waypoints;

  Vector3 scale;
  Header header;
  string state;

  string mode_;
  bool   boost_;
  double wall_clearance_;
  double group_radius_;
  double max_range_;
  double min_range_;
  double target_dir_;
  double boost_speed_;
  double max_speed_;
  double inter_speed_;
  double min_speed_;
  double slow_down_gap_dist_;
  double slow_down_steer_angle_;
  double look_ahead_;
  double kp_pure_pursuit_;

  void onTimer();
  bool checkData();
  void updateClosest();
  double calcSteerCmd();
  double calcAccCmd();
  void pubGapMarker(pair<double, double> target_gap, vector<double> color);
  void preproc_scan();
  void update_range_thres();
  void find_gaps();
  void find_target_gaps();
  void visualize_path_points();
};

}  // namespace smart_gap_follower

