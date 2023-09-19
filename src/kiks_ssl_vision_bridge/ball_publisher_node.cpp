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


#include "kiks_ssl_vision_bridge/ball_publisher_node.hpp"

#include <string>

namespace kiks::ssl_vision_bridge
{

std::string BallPublisherNode::default_name() {return "ssl_vision_bridge_ball_publisher";}

BallPublisherNode::BallPublisherNode(const rclcpp::NodeOptions & options)
: BallPublisherNode(std::make_shared<rclcpp::Node>(default_name(), options))
{
}

BallPublisherNode::BallPublisherNode(
  const std::string & node_name,
  const rclcpp::NodeOptions & options)
: BallPublisherNode(std::make_shared<rclcpp::Node>(node_name, options))
{
}

BallPublisherNode::BallPublisherNode(
  const std::string & node_name, const std::string & node_namespace,
  const rclcpp::NodeOptions & options)
: BallPublisherNode(std::make_shared<rclcpp::Node>(node_name, node_namespace, options))
{
}

BallPublisherNode::BallPublisherNode(rclcpp::Node::SharedPtr node)
: RosNodeBase(std::move(node))
{
  this->add_parameter<bool>(
    "tf.enable", false, [this](const auto & param) {
      if (param.as_bool()) {
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*node_);
      } else {
        tf_broadcaster_.reset();
      }
    });

  const std::string ns = node_->get_namespace();
  const std::string ns_with_slash = ns.back() == '/' ? ns : ns + '/';
  this->add_parameter<std::string>(
    "ball.frame_id", "vision/ball", [this, ns_with_slash](const auto & param) {
      auto str = param.as_string();
      ball_publisher_ =
      node_->create_publisher<PointMsg>(ns_with_slash + str, this->get_dynamic_qos());
      tf_msg_.child_frame_id = str;
    });

  this->add_parameter<std::string>(
    "map.frame_id", "map", [this](const auto & param) {
      auto str = param.as_string();
      ball_msg_.header.frame_id = str;
      tf_msg_.header.frame_id = str;
    });
}

void BallPublisherNode::publish_ball(const TimeMsg & stamp, const SSL_DetectionBall & ball)
{
  ball_msg_.header.stamp = stamp;
  ball_msg_.point.x = ball.x() * 0.001;
  ball_msg_.point.y = ball.y() * 0.001;
  ball_msg_.point.z = ball.z() * 0.001;
  ball_publisher_->publish(ball_msg_);
  if (tf_broadcaster_) {
    tf_msg_.header.stamp = stamp;
    tf_msg_.transform.translation.x = ball_msg_.point.x;
    tf_msg_.transform.translation.y = ball_msg_.point.y;
    tf_msg_.transform.translation.z = ball_msg_.point.z;
    tf_broadcaster_->sendTransform(tf_msg_);
  }
}

}  // namespace kiks::ssl_vision_bridge
