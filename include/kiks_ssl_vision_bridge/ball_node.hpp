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

#ifndef KIKS_SSL_VISION_BRIDGE_BALL_NODE_HPP_
#define KIKS_SSL_VISION_BRIDGE_BALL_NODE_HPP_

#include <string>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/node.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "kiks_ssl_vision_bridge/msg/vision_detection.hpp"

namespace kiks::ssl_vision_bridge
{

class BallNode
{
public:
  static std::string default_name();
  
  BallNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  BallNode(
    const std::string &node_name,
    const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  BallNode(
    const std::string &node_name,
    const std::string &node_namespace,
    const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    
  BallNode(rclcpp::Node::SharedPtr node);

  operator rclcpp::Node::SharedPtr() {
    return node_;
  }

private:
  using VisionDetectionMsg = kiks_ssl_vision_bridge::msg::VisionDetection;
  using PointMsg = geometry_msgs::msg::PointStamped;
  using TfMsg = geometry_msgs::msg::TransformStamped;

  void extract(VisionDetectionMsg::ConstSharedPtr vision_detection_msg);

  void extract_with_tf(VisionDetectionMsg::ConstSharedPtr vision_detection_msg);

  void publish_ball(VisionDetectionMsg::ConstSharedPtr vision_detection_msg);

  void broadcast_ball_tf(VisionDetectionMsg::ConstSharedPtr vision_detection_msg);

  rclcpp::Node::SharedPtr node_;
  TfMsg tf_msg_;
  rclcpp::Publisher<PointMsg>::SharedPtr ball_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<VisionDetectionMsg>::SharedPtr vision_detection_subscription_;
};

} // namespace kiks::ssl_vision_bridge

#endif // #ifndef KIKS_SSL_VISION_BRIDGE_BALL_NODE_HPP_
