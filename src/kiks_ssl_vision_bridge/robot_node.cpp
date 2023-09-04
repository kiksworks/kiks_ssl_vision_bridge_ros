// Copyright 2023 KIKS
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.


#include "kiks_ssl_vision_bridge/robot_node.hpp"

namespace kiks::ssl_vision_bridge
{

template<class T>
T declare_parameter_if_not_set(
  rclcpp::Node::SharedPtr node, const std::string & name, const T & default_value,
  const std::string & parameter_namespace = "")
{
  auto value = node->has_parameter(name) ? node->get_parameter(name).get_value<T>() :
    node->declare_parameter<T>(name, default_value);
  if (!parameter_namespace.empty()) {
    value = declare_parameter_if_not_set(node, parameter_namespace + "." + name, value);
  }
  return value;
}

using namespace std::chrono_literals;

std::string RobotNode::default_name() {return "ssl_bridge_vision_robot";}

RobotNode::RobotNode(const rclcpp::NodeOptions & options)
: RobotNode(std::make_shared<rclcpp::Node>(default_name(), options))
{
}

RobotNode::RobotNode(const std::string & node_name, const rclcpp::NodeOptions & options)
: RobotNode(std::make_shared<rclcpp::Node>(node_name, options))
{
}

RobotNode::RobotNode(
  const std::string & node_name, const std::string & node_namespace,
  const rclcpp::NodeOptions & options)
: RobotNode(std::make_shared<rclcpp::Node>(node_name, node_namespace, options))
{
}

RobotNode::RobotNode(rclcpp::Node::SharedPtr node)
: RosNodeBase(std::move(node))
{
  this->add_parameter<std::string>(
    "team_color", "yellow", [this](const auto & param) {
      team_is_yellow_ = param.as_string() == "yellow";
    });
  this->add_parameter<std::int64_t>(
    "robot_id", 0, [this](const auto & param) {robot_id_ = param.as_int();});
  this->add_parameter<bool>(
    "tf.enable", false, [this](const auto & param) {
      tf_enable_ = param.as_bool();
      if (tf_enable_) {
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*node_);
      } else {
        tf_broadcaster_.reset();
      }
    });

  const auto tf_namespace =
    node_->get_sub_namespace().empty() ? "" : node_->get_sub_namespace() + "/";
  this->add_parameter<std::string>(
    "frame_id", std::string("vision_footprint"), [this, tf_namespace](const auto & param) {
      tf_msg_.child_frame_id = tf_namespace + param.as_string();
    });

  robot_publisher_ =
    node_->create_publisher<PoseMsg>("vision_footprint", rclcpp::QoS(4).best_effort());

  const auto topic_namespace =
    std::string(node_->get_namespace()) == "/" ? "/" : std::string(node_->get_namespace()) + "/";
  vision_detection_subscription_ = node_->create_subscription<VisionDetectionMsg>(
    topic_namespace + "vision_detection", rclcpp::QoS(4).best_effort(),
    std::bind(&RobotNode::convert, this, std::placeholders::_1));
}

void RobotNode::convert(VisionDetectionMsg::ConstSharedPtr vision_detection_msg)
{
  auto & robots_with_id =
    team_is_yellow_ ? vision_detection_msg->yellow_robots : vision_detection_msg->blue_robots;

  auto stamp = node_->now();

  for (auto & robot_with_id : robots_with_id) {
    if (robot_with_id.id == robot_id_) {
      PoseMsg pose_msg;
      pose_msg.header = vision_detection_msg->header;
      pose_msg.pose = robot_with_id.pose;
      robot_publisher_->publish(pose_msg);
      if (tf_enable_) {
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

}  // namespace kiks::ssl_vision_bridge
