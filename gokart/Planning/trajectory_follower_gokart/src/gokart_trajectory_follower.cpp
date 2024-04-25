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

#include "trajectory_follower_gokart/gokart_trajectory_follower.hpp"
//#include <motion_utils/motion_utils.hpp>
#include <motion_utils/trajectory/trajectory.hpp>
#include <tier4_autoware_utils/geometry/pose_deviation.hpp>

#include <algorithm>

using namespace std;

namespace gokart_trajectory_follower
{

using motion_utils::findNearestIndex;
using tier4_autoware_utils::calcLateralDeviation;
using tier4_autoware_utils::calcYawDeviation;

GokartTrajectoryFollower::GokartTrajectoryFollower(const rclcpp::NodeOptions & options)
: Node("gokart_trajectory_follower", options)
{
  drive_cmd_ = create_publisher<AckermannDriveStamped>("/drive", 1);
  traj_marker_pub_ = create_publisher<Marker>("/wp_marker", 1);
  goal_marker_pub_ = create_publisher<Marker>("/goal_marker", 1);

  sub_kinematics_ = create_subscription<Odometry>(
    "input/kinematics", 1, [this](const Odometry::SharedPtr msg) { odometry_ = msg; });
  sub_trajectory_ = create_subscription<Trajectory>(
    "input/trajectory", 1, [this](const Trajectory::SharedPtr msg) { trajectory_ = msg; });

  use_external_target_vel_ = declare_parameter<bool>("use_external_target_vel", false);
  external_target_vel_ = declare_parameter<float>("external_target_vel", 0.0);
  lateral_deviation_ = declare_parameter<float>("lateral_deviation", 0.0);

  using namespace std::literals::chrono_literals;
  timer_ = rclcpp::create_timer(
    this, get_clock(), 30ms, std::bind(&GokartTrajectoryFollower::onTimer, this));
}

void GokartTrajectoryFollower::createTrajectoryMarker(){
    scale.x = 0.1;
    scale.y = 0.1;
    scale.z = 0.1;

    header.frame_id = "map";

    marker.type = marker.POINTS;
    marker.pose = pose;
    marker.scale = scale;
    marker.header = header;
    marker.points.clear();
    marker.lifetime = rclcpp::Duration::from_nanoseconds(0.03 * 1e9);

    for(int i = 0; i < (int)trajectory_->points.size(); i++){
      point.x = trajectory_->points[i].pose.position.x;
      point.y = trajectory_->points[i].pose.position.y;
      point.z = 0.0;
      marker.points.push_back(point);
    }

    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    traj_marker_pub_->publish(marker);

    scale.x = 0.2;
    scale.y = 0.2;
    scale.z = 0.2;

    goal_marker.type = goal_marker.POINTS;
    goal_marker.pose = pose;
    goal_marker.scale = scale;
    goal_marker.header = header;
    goal_marker.lifetime = rclcpp::Duration::from_nanoseconds(0.03 * 1e9);

    point.x = closest_traj_point_.pose.position.x;
    point.y = closest_traj_point_.pose.position.y;
    point.z = 0.0;
    
    goal_marker.points.clear();
    goal_marker.points.push_back(point);

    goal_marker.color.r = 0.0;
    goal_marker.color.g = 0.0;
    goal_marker.color.b = 1.0;
    goal_marker.color.a = 1.0;

    goal_marker_pub_->publish(goal_marker);
}

void GokartTrajectoryFollower::onTimer()
{
  if (!checkData()) {
    // RCLCPP_INFO(get_logger(), "data not ready!!");
    return;
  }

  updateClosest();
  createTrajectoryMarker();

  AckermannControlCommand cmd;
  cmd.stamp = cmd.lateral.stamp = cmd.longitudinal.stamp = get_clock()->now();
  cmd.lateral.steering_tire_angle = static_cast<float>(calcSteerCmd());
  cmd.longitudinal.speed = use_external_target_vel_ ? static_cast<float>(external_target_vel_)
                                                    : closest_traj_point_.longitudinal_velocity_mps;
  cmd.longitudinal.acceleration = static_cast<float>(calcAccCmd());

  ackermann_msgs::msg::AckermannDriveStamped ackermann_msg;
  ackermann_msg.drive.speed = cmd.longitudinal.speed * 0.2;
  ackermann_msg.drive.steering_angle = cmd.lateral.steering_tire_angle * 10;
  drive_cmd_->publish(ackermann_msg);

  cout << "velocity: " << ackermann_msg.drive.speed << "m/s"<< endl;
}

void GokartTrajectoryFollower::updateClosest()
{
  const auto closest = findNearestIndex(trajectory_->points, odometry_->pose.pose.position);
  closest_traj_point_ = trajectory_->points.at(closest);
}

double GokartTrajectoryFollower::calcSteerCmd()
{
  const auto lat_err =
    calcLateralDeviation(closest_traj_point_.pose, odometry_->pose.pose.position) -
    lateral_deviation_;
  const auto yaw_err = calcYawDeviation(closest_traj_point_.pose, odometry_->pose.pose);

  // linearized pure_pursuit control
  constexpr auto wheel_base = 0.25;
  constexpr auto lookahead_time = 3.0;
  constexpr auto min_lookahead = 3.0;
  const auto lookahead = min_lookahead + lookahead_time * std::abs(odometry_->twist.twist.linear.x);
  const auto kp = 25.0 * wheel_base / (lookahead * lookahead);
  const auto kd = 5.0 * wheel_base / lookahead;

  constexpr auto steer_lim = 1.0;

  const auto steer = std::clamp(-kp * lat_err - kd * yaw_err, -steer_lim, steer_lim);
  
  return steer;
}

double GokartTrajectoryFollower::calcAccCmd()
{
  const auto traj_vel = static_cast<double>(closest_traj_point_.longitudinal_velocity_mps);
  const auto ego_vel = odometry_->twist.twist.linear.x;
  const auto target_vel = use_external_target_vel_ ? external_target_vel_ : traj_vel;
  const auto vel_err = ego_vel - target_vel;

  // P feedback
  constexpr auto kp = 0.5;
  constexpr auto acc_lim = 2.0;

  const auto acc = std::clamp(-kp * vel_err, -acc_lim, acc_lim);
  RCLCPP_DEBUG(get_logger(), "vel_err = %f, acc = %f", vel_err, acc);
  return acc;
}

bool GokartTrajectoryFollower::checkData() { return (trajectory_ && odometry_); }

}  // namespace gokart_trajectory_follower

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(gokart_trajectory_follower::GokartTrajectoryFollower)
