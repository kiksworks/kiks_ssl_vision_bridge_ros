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


#ifndef KIKS_SSL_VISION_BRIDGE__ROBOT_NODE_HPP_
#define KIKS_SSL_VISION_BRIDGE__ROBOT_NODE_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/node.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "kiks_ssl_vision_bridge/msg/vision_detection.hpp"
#include "kiks_ssl_vision_bridge/ros_node_base.hpp"

namespace kiks::ssl_vision_bridge
{

class RobotNode : public RosNodeBase
{
public:
  static std::string default_name();

  explicit RobotNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  RobotNode(
    const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  RobotNode(
    const std::string & node_name, const std::string & node_namespace,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  explicit RobotNode(rclcpp::Node::SharedPtr node);

private:
  using VisionDetectionMsg = kiks_ssl_vision_bridge::msg::VisionDetection;
  using PoseMsg = geometry_msgs::msg::PoseStamped;
  using TfMsg = geometry_msgs::msg::TransformStamped;

  template<bool kTfEnable>
  void extract(VisionDetectionMsg::ConstSharedPtr vision_detection_msg);

  bool team_is_yellow_;
  int robot_id_;
  TfMsg tf_msg_;
  rclcpp::Publisher<PoseMsg>::SharedPtr robot_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<VisionDetectionMsg>::SharedPtr vision_detection_subscription_;
};

}  // namespace kiks::ssl_vision_bridge

#endif  // KIKS_SSL_VISION_BRIDGE__ROBOT_NODE_HPP_
