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


#ifndef KIKS_SSL_VISION_BRIDGE__ROBOT_PUBLISHER_NODE_HPP_
#define KIKS_SSL_VISION_BRIDGE__ROBOT_PUBLISHER_NODE_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "kiks_ssl_vision_bridge/ros_node_base.hpp"
#include "messages_robocup_ssl_detection.pb.h"

namespace kiks::ssl_vision_bridge
{

class RobotPublisherNode : public RosNodeBase
{
public:
  using TimeMsg = builtin_interfaces::msg::Time;

  static inline std::string default_name();

  explicit RobotPublisherNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  RobotPublisherNode(
    const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  RobotPublisherNode(
    const std::string & node_name, const std::string & node_namespace,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  explicit RobotPublisherNode(rclcpp::Node::SharedPtr node);

  void publish_robot(const TimeMsg& stamp, const SSL_DetectionRobot& robot);

private:
  using PoseMsg = geometry_msgs::msg::PoseStamped;
  using TfMsg = geometry_msgs::msg::TransformStamped;

  int robot_id_;
  PoseMsg pose_msg_;
  TfMsg tf_msg_;
  rclcpp::Publisher<PoseMsg>::SharedPtr robot_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace kiks::ssl_vision_bridge

#endif  // KIKS_SSL_VISION_BRIDGE__ROBOT_PUBLISHER_NODE_HPP_
