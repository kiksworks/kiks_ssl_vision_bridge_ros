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


#include "kiks_ssl_vision_bridge/ball_node.hpp"

#include <chrono>
#include <string>

namespace kiks::ssl_vision_bridge
{

using namespace std::chrono_literals;

std::string BallNode::default_name() {return "ssl_vision_bridge_ball";}

BallNode::BallNode(const rclcpp::NodeOptions & options)
: BallNode(std::make_shared<rclcpp::Node>(default_name(), options))
{
}

BallNode::BallNode(const std::string & node_name, const rclcpp::NodeOptions & options)
: BallNode(std::make_shared<rclcpp::Node>(node_name, options))
{
}

BallNode::BallNode(
  const std::string & node_name, const std::string & node_namespace,
  const rclcpp::NodeOptions & options)
: BallNode(std::make_shared<rclcpp::Node>(node_name, node_namespace, options))
{
}

BallNode::BallNode(rclcpp::Node::SharedPtr node)
: RosNodeBase(std::move(node))
{
  const auto topic_namespace =
    std::string(node_->get_namespace()) == "/" ? "/" : std::string(node_->get_namespace()) + "/";
  this->add_parameter<bool>(
    "tf.enable", false, [this, topic_namespace](const auto & param) {
      if (param.as_bool()) {
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*node_);
        vision_detection_subscription_ = node_->create_subscription<VisionDetectionMsg>(
          topic_namespace + "vision_detection", rclcpp::QoS(4).best_effort(),
          std::bind(&BallNode::extract_with_tf, this, std::placeholders::_1));
      } else {
        tf_broadcaster_.reset();
        vision_detection_subscription_ = node_->create_subscription<VisionDetectionMsg>(
          topic_namespace + "vision_detection", rclcpp::QoS(4).best_effort(),
          std::bind(&BallNode::extract, this, std::placeholders::_1));
      }
    });

  const auto tf_namespace =
    node_->get_sub_namespace().empty() ? "" : node_->get_sub_namespace() + "/";
  this->add_parameter<std::string>(
    "frame_id", "ball", [this, tf_namespace](const auto & param) {
      tf_msg_.child_frame_id = tf_namespace + param.as_string();
    });

  ball_publisher_ = node_->create_publisher<PointMsg>("ball", rclcpp::QoS(4).best_effort());
}

void BallNode::extract(VisionDetectionMsg::ConstSharedPtr vision_detection_msg)
{
  if (!vision_detection_msg->balls.size()) {
    return;
  }
  publish_ball(vision_detection_msg);
}

void BallNode::extract_with_tf(VisionDetectionMsg::ConstSharedPtr vision_detection_msg)
{
  if (!vision_detection_msg->balls.size()) {
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

}  // namespace kiks::ssl_vision_bridge
