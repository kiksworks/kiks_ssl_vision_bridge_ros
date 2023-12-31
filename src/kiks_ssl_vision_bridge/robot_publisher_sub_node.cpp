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


#include "kiks_ssl_vision_bridge/robot_publisher_sub_node.hpp"

#include <cmath>

namespace kiks::ssl_vision_bridge
{

std::string RobotPublisherSubNode::default_name() {return "ssl_vision_bridge_robot_publisher";}

RobotPublisherSubNode::RobotPublisherSubNode(const rclcpp::NodeOptions & options)
: RobotPublisherSubNode(std::make_shared<rclcpp::Node>(default_name(), options))
{
}

RobotPublisherSubNode::RobotPublisherSubNode(
  const std::string & node_name,
  const rclcpp::NodeOptions & options)
: RobotPublisherSubNode(std::make_shared<rclcpp::Node>(node_name, options))
{
}

RobotPublisherSubNode::RobotPublisherSubNode(
  const std::string & node_name, const std::string & node_namespace,
  const rclcpp::NodeOptions & options)
: RobotPublisherSubNode(std::make_shared<rclcpp::Node>(node_name, node_namespace, options))
{
}

RobotPublisherSubNode::RobotPublisherSubNode(rclcpp::Node::SharedPtr node)
: ExpandedSubNode(std::move(node))
{
  // Parameter of tf
  this->add_param<bool>(
    "tf.enable", false, [this](const auto & param) {
      if (param.as_bool()) {
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(**this);
      } else {
        tf_broadcaster_.reset();
      }
    });
  // Parameter of robot frame_id
  const auto tf_namespace =
    (*this)->get_sub_namespace().empty() ? "" : (*this)->get_sub_namespace() + "/";
  this->add_param<std::string>(
    "robot.frame_id", "vision/base_footprint", [this, tf_namespace](const auto & param) {
      auto str = param.as_string();
      robot_publisher_ =
      (*this)->create_publisher<PoseMsg>(str, this->get_dynamic_qos());
      tf_msg_.child_frame_id = tf_namespace + param.as_string();
    });
  // Parameter of map frame_id
  this->add_param<std::string>(
    "map.frame_id", "map", [this](const auto & param) {
      auto str = param.as_string();
      pose_msg_.header.frame_id = tf_msg_.header.frame_id = param.as_string();
    });
}

void RobotPublisherSubNode::publish_robot(const TimeMsg & stamp, const SSL_DetectionRobot & robot)
{
  pose_msg_.header.stamp = stamp;
  pose_msg_.pose.pose.position.x = robot.x() * 0.001;
  pose_msg_.pose.pose.position.y = robot.y() * 0.001;
  // Quaternion calculations
  // - Since the rotation vector vector is (0, 0, 1), it becomes
  //   x=0 y=0 z=sin(orientation/2) w=cos(orientation/2)
  const auto theta = robot.orientation() * 0.5;
  pose_msg_.pose.pose.orientation.z = std::sin(theta);
  pose_msg_.pose.pose.orientation.w = std::cos(theta);
  robot_publisher_->publish(pose_msg_);

  if (tf_broadcaster_) {
    tf_msg_.header.stamp = stamp;
    tf_msg_.transform.translation.x = pose_msg_.pose.pose.position.x;
    tf_msg_.transform.translation.y = pose_msg_.pose.pose.position.y;
    tf_msg_.transform.rotation.z = pose_msg_.pose.pose.orientation.z;
    tf_msg_.transform.rotation.w = pose_msg_.pose.pose.orientation.w;
    tf_broadcaster_->sendTransform(tf_msg_);
  }
}

}  // namespace kiks::ssl_vision_bridge
