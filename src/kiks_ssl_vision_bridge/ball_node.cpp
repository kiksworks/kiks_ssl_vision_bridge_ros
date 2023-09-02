// Copyright 2023 KIKS.
//
// Licensed under the GNU GENERAL PUBLIC LICENSE, Version 3.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.gnu.org/licenses/gpl-3.0.html
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "kiks_ssl_vision_bridge/ball_node.hpp"

#include <chrono>
#include <string>

namespace kiks::ssl_vision_bridge
{

template <class T>
T declare_parameter_if_not_set(rclcpp::Node::SharedPtr node, const std::string& name, const T& default_value, const std::string& parameter_namespace = ""){
  auto value = node->has_parameter(name) ?
    node->get_parameter(name).get_value<T>() :
    node->declare_parameter<T>(name, default_value);
  if(!parameter_namespace.empty()) {
    value = declare_parameter_if_not_set(node, parameter_namespace + "." + name, value);
  }
  return value;
};

using namespace std::chrono_literals;

std::string BallNode::default_name()
{
  return "ssl_bridge_vision_ball";
}

BallNode::BallNode(const rclcpp::NodeOptions &options) :
  BallNode(std::make_shared<rclcpp::Node>(default_name(), options)) {}

BallNode::BallNode(
  const std::string &node_name,
  const rclcpp::NodeOptions &options) :
  BallNode(std::make_shared<rclcpp::Node>(node_name, options)) {}

BallNode::BallNode(
  const std::string &node_name,
  const std::string &node_namespace, 
  const rclcpp::NodeOptions &options) :
  BallNode(std::make_shared<rclcpp::Node>(node_name, node_namespace, options)) {}
  
BallNode::BallNode(rclcpp::Node::SharedPtr node) :
  node_(std::move(node))
{
  const bool sub_namespace_is_empty = node_->get_sub_namespace().empty();
  const auto tf_namespace = sub_namespace_is_empty ? "" : node_->get_sub_namespace() + "/";
  const auto topic_namespace = std::string(node_->get_namespace()) == "/" ? "/" : std::string(node_->get_namespace()) + "/";

  auto init = [this](const std::string& name, const auto& value) {
    return declare_parameter_if_not_set(node_, name, value, node_->get_sub_namespace());
  };
  
  auto tf_enable = init("tf.enable", false);
  tf_msg_.child_frame_id = tf_namespace + init("frame_id", std::string("ball"));

  ball_publisher_ = 
    node_->create_publisher<PointMsg>("ball", rclcpp::QoS(4).best_effort());
  if(tf_enable) {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*node_);
    vision_detection_subscription_ = node_->create_subscription<VisionDetectionMsg>(
      topic_namespace + "vision_detection",
      rclcpp::QoS(4).best_effort(),
      std::bind(&BallNode::extract_with_tf, this, std::placeholders::_1));
  }
  else {
    vision_detection_subscription_ = node_->create_subscription<VisionDetectionMsg>(
      topic_namespace + "vision_detection",
      rclcpp::QoS(4).best_effort(),
      std::bind(&BallNode::extract, this, std::placeholders::_1));
  }
}

void BallNode::extract(VisionDetectionMsg::ConstSharedPtr vision_detection_msg)
{
  if(!vision_detection_msg->balls.size()) {
    return;
  }
  publish_ball(vision_detection_msg);
}

void BallNode::extract_with_tf(VisionDetectionMsg::ConstSharedPtr vision_detection_msg)
{
  if(!vision_detection_msg->balls.size()) {
    return;
  }
  publish_ball(vision_detection_msg);
  broadcast_ball_tf(vision_detection_msg);
}

void BallNode::publish_ball(VisionDetectionMsg::ConstSharedPtr vision_detection_msg)
{
  PointMsg ball_msg;
  ball_msg.header = vision_detection_msg->header;
  ball_msg.point = vision_detection_msg->balls[0];
  ball_publisher_->publish(ball_msg);
}

void BallNode::broadcast_ball_tf(VisionDetectionMsg::ConstSharedPtr vision_detection_msg)
{
  tf_msg_.header = vision_detection_msg->header;
  tf_msg_.transform.translation.x = vision_detection_msg->balls[0].x;
  tf_msg_.transform.translation.y = vision_detection_msg->balls[0].y;
  tf_msg_.transform.translation.z = vision_detection_msg->balls[0].z;
  tf_msg_.transform.rotation.w = 1;
  tf_broadcaster_->sendTransform(tf_msg_);
}

} // namespace kiks::ssl_vision_bridge
