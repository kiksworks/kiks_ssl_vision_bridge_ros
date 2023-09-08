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


#ifndef KIKS_SSL_VISION_BRIDGE__BALL_NODE_HPP_
#define KIKS_SSL_VISION_BRIDGE__BALL_NODE_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "kiks_ssl_vision_bridge/msg/vision_detection.hpp"
#include "kiks_ssl_vision_bridge/ros_node_base.hpp"

namespace kiks::ssl_vision_bridge
{

class BallNode : public RosNodeBase
{
public:
  static std::string default_name();

  explicit BallNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  BallNode(
    const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  BallNode(
    const std::string & node_name, const std::string & node_namespace,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  explicit BallNode(rclcpp::Node::SharedPtr node);

private:
  using VisionDetectionMsg = kiks_ssl_vision_bridge::msg::VisionDetection;
  using PointMsg = geometry_msgs::msg::PointStamped;
  using TfMsg = geometry_msgs::msg::TransformStamped;

  void extract(VisionDetectionMsg::ConstSharedPtr vision_detection_msg);

  void extract_with_tf(VisionDetectionMsg::ConstSharedPtr vision_detection_msg);

  inline void publish_ball(VisionDetectionMsg::ConstSharedPtr vision_detection_msg);

  inline void broadcast_ball_tf(VisionDetectionMsg::ConstSharedPtr vision_detection_msg);

  TfMsg tf_msg_;
  rclcpp::Publisher<PointMsg>::SharedPtr ball_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<VisionDetectionMsg>::SharedPtr vision_detection_subscription_;
};

}  // namespace kiks::ssl_vision_bridge

#endif  // KIKS_SSL_VISION_BRIDGE__BALL_NODE_HPP_
