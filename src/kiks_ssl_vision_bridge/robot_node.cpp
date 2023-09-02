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

#include "kiks_ssl_vision_bridge/robot_node.hpp"

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

std::string RobotNode::default_name()
{
  return "ssl_bridge_vision_robot";
}

RobotNode::RobotNode(const rclcpp::NodeOptions &options) :
  RobotNode(std::make_shared<rclcpp::Node>(default_name(), options)) {}

RobotNode::RobotNode(
  const std::string &node_name,
  const rclcpp::NodeOptions &options) :
  RobotNode(std::make_shared<rclcpp::Node>(node_name, options)) {}

RobotNode::RobotNode(
  const std::string &node_name,
  const std::string &node_namespace, 
  const rclcpp::NodeOptions &options) :
  RobotNode(std::make_shared<rclcpp::Node>(node_name, node_namespace, options)) {}
  
RobotNode::RobotNode(rclcpp::Node::SharedPtr node) :
  node_(std::move(node))
{
  const bool sub_namespace_is_empty = node_->get_sub_namespace().empty();
  const auto tf_namespace = sub_namespace_is_empty ? "" : node_->get_sub_namespace() + "/";
  const auto topic_namespace = std::string(node_->get_namespace()) == "/" ? "/" : std::string(node_->get_namespace()) + "/";

  auto init = [this](const std::string& name, const auto& value) {
    return declare_parameter_if_not_set(node_, name, value, node_->get_sub_namespace());
  };
  
  team_is_yellow_ = init("team_color", std::string("yellow")) == "yellow";
  robot_id_ = init("robot_id", 0);
  tf_enable_ = init("tf.enable", false);
  tf_msg_.child_frame_id = tf_namespace + init("frame_id", std::string("vision_footprint"));

  robot_publisher_ = 
    node_->create_publisher<PoseMsg>("vision_footprint", rclcpp::QoS(4).best_effort());
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*node_);
  vision_detection_subscription_ = node_->create_subscription<VisionDetectionMsg>(
    topic_namespace + "vision_detection", 
    rclcpp::QoS(4).best_effort(), 
    std::bind(&RobotNode::convert, this, std::placeholders::_1));
}

void RobotNode::convert(VisionDetectionMsg::ConstSharedPtr vision_detection_msg)
{
  auto& robots_with_id = team_is_yellow_ ?
    vision_detection_msg->yellow_robots :
    vision_detection_msg->blue_robots;
  
  auto stamp = node_->now();
  
  for(auto& robot_with_id : robots_with_id) {
    if(robot_with_id.id == robot_id_) {
      PoseMsg pose_msg;
      pose_msg.header = vision_detection_msg->header;
      pose_msg.pose = robot_with_id.pose;
      robot_publisher_->publish(pose_msg);
      if(tf_enable_) {
        tf_msg_.header = vision_detection_msg->header;
        tf_msg_.transform.translation.x = robot_with_id.pose.position.x;
        tf_msg_.transform.translation.y = robot_with_id.pose.position.y;
        tf_msg_.transform.translation.z = robot_with_id.pose.position.z;
        tf_msg_.transform.rotation = robot_with_id.pose.orientation;
        tf_broadcaster_->sendTransform(tf_msg_);
      }
    }
  }
}

} // namespace kiks::ssl_vision_bridge
