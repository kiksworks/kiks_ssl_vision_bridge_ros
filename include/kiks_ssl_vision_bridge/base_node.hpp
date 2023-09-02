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

#ifndef KIKS_SSL_VISION_BRIDGE_BASE_NODE_HPP_
#define KIKS_SSL_VISION_BRIDGE_BASE_NODE_HPP_

#include "QUdpSocket"
#include "rclcpp/node.hpp"

#include "kiks_ssl_vision_bridge/msg/vision_detection.hpp"

namespace kiks::ssl_vision_bridge
{

class BaseNode
{
public:
  BaseNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  BaseNode(
    const std::string &node_name,
    const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  BaseNode(
    const std::string &node_name,
    const std::string &node_namespace,
    const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    
  BaseNode(rclcpp::Node::SharedPtr node);

  operator rclcpp::Node::SharedPtr() {
    return node_;
  }

private:
  using VisionDetectionMsg = kiks_ssl_vision_bridge::msg::VisionDetection;
  using PoseWithIdMsg = kiks_ssl_vision_bridge::msg::PoseWithId;
  using PointMsg = geometry_msgs::msg::Point;

  rclcpp::Node::SharedPtr node_;
  QUdpSocket udp_socket_;
  std::string map_frame_id_;
  rclcpp::Publisher<VisionDetectionMsg>::SharedPtr vision_detection_publisher_;
  rclcpp::TimerBase::SharedPtr receiver_;

  void receive();

  void publish_vision_detection(const QByteArray& recv_byte_arr);
};

} // namespace kiks::ssl_vision_bridge

#endif // #ifndef KIKS_SSL_VISION_BRIDGE_BASE_NODE_HPP_
