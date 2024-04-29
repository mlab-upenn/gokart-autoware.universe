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

#include "classic_grass_detection/classic_grass_detection_node.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "cv_bridge/cv_bridge.h"
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

namespace classic_grass_detection
{

ClassicGrassDetectionNode::ClassicGrassDetectionNode(const rclcpp::NodeOptions & node_options)
: Node("classic_grass_detection", node_options)
{
  // Parameter Server
  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&ClassicGrassDetectionNode::onSetParam, this, std::placeholders::_1));

  // Node Parameter
  node_param_.bev_width = declare_parameter<int>("bev_width");
  node_param_.bev_height = declare_parameter<int>("bev_height");
  node_param_.bev_origin_x = declare_parameter<int>("bev_origin_x");
  node_param_.bev_origin_y = declare_parameter<int>("bev_origin_y");
  node_param_.px_dist = declare_parameter<double>("px_dist");

  // Subscriber
  sub_image_ = create_subscription<sensor_msgs::msg::Image>(
    "~/input/image", rclcpp::QoS{1},
    std::bind(&ClassicGrassDetectionNode::onImage, this, std::placeholders::_1));

  // Publisher
  pub_image_ = create_publisher<sensor_msgs::msg::Image>("~/output/image", 1);
}

void ClassicGrassDetectionNode::onImage(const sensor_msgs::msg::Image::ConstSharedPtr image_)
{
  sensor_msgs::msg::Image output_image;
  output_image.data = image_->data;
  /*
  DetectedObjects output_objects;
  output_objects.header = objects_data_->header;
  std::vector<DetectedObject> objects = objects_data_->objects;

  std::vector<bool> used_flags(objects.size(), false);

  auto func = [](DetectedObject const & lhs, DetectedObject const & rhs) {
    return get_distance(lhs) < get_distance(rhs);
  };
  std::sort(objects.begin(), objects.end(), func);

  for (size_t i = 0; i < objects.size(); i++) {
    if (used_flags.at(i) == true) {
      continue;
    }

    std::vector<DetectedObject> clustered_objects;
    used_flags.at(i) = true;
    clustered_objects.emplace_back(objects.at(i));

    for (size_t j = i; j < objects.size(); ++j) {
      if (used_flags.at(j) == false && isSameObject(objects.at(i), objects.at(j))) {
        used_flags.at(j) = true;
        clustered_objects.emplace_back(objects.at(j));
      }
    }

    // clustering
    DetectedObject clustered_output_object;
    if (clustered_objects.size() == 1) {
      clustered_output_object = clustered_objects.at(0);
    } else {
      auto func_max_confidence = [](const DetectedObject & a, const DetectedObject & b) {
        return a.existence_probability < b.existence_probability;
      };
      auto iter = std::max_element(
        std::begin(clustered_objects), std::end(clustered_objects), func_max_confidence);

      // class label
      clustered_output_object.existence_probability = iter->existence_probability;
      clustered_output_object.classification = iter->classification;

      // kinematics
      clustered_output_object.kinematics = iter->kinematics;

      auto & pose = clustered_output_object.kinematics.pose_with_covariance.pose;
      auto func_sum_x = [](const double & a, const DetectedObject & b) {
        return a + b.kinematics.pose_with_covariance.pose.position.x;
      };
      pose.position.x =
        std::accumulate(
          std::begin(clustered_objects), std::end(clustered_objects), 0.0, func_sum_x) /
        clustered_objects.size();
      auto func_sum_y = [](const double & a, const DetectedObject & b) {
        return a + b.kinematics.pose_with_covariance.pose.position.y;
      };
      pose.position.y =
        std::accumulate(
          std::begin(clustered_objects), std::end(clustered_objects), 0.0, func_sum_y) /
        clustered_objects.size();
      pose.position.z = iter->kinematics.pose_with_covariance.pose.position.z;

      // Shape
      clustered_output_object.shape = iter->shape;
    }

    // Fixed label correction
    if (node_param_.is_fixed_label) {
      clustered_output_object.classification.at(0).label =
        object_recognition_utils::toLabel(node_param_.fixed_label);
    }

    // Fixed size correction
    if (node_param_.is_fixed_size) {
      clustered_output_object.shape.type = autoware_auto_perception_msgs::msg::Shape::BOUNDING_BOX;
      clustered_output_object.shape.dimensions.x = node_param_.size_x;
      clustered_output_object.shape.dimensions.y = node_param_.size_y;
      clustered_output_object.shape.dimensions.z = node_param_.size_z;
    }
    output_objects.objects.emplace_back(clustered_output_object);
  }
  */
  
  pub_image_->publish(output_image);
  
}

rcl_interfaces::msg::SetParametersResult ClassicGrassDetectionNode::onSetParam(
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
      update_param(params, "bev_origin_x", p.bev_origin_x);
      update_param(params, "bev_origin_y", p.bev_origin_y);
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
RCLCPP_COMPONENTS_REGISTER_NODE(classic_grass_detection::ClassicGrassDetectionNode)
