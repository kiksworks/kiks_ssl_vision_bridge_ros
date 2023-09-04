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

#include <cstdint>

#include "QUdpSocket"
#include "rclcpp/node.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"

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
  using MapMsg = nav_msgs::msg::OccupancyGrid;
  using MapDataItr = std::vector<std::int8_t>::iterator;

  rclcpp::Node::SharedPtr node_;
  QUdpSocket udp_socket_;
  std::string map_frame_id_;
  rclcpp::Publisher<VisionDetectionMsg>::SharedPtr vision_detection_publisher_;
  bool map_enable_;
  MapMsg map_msg_;
  double map_wall_width_;
  rclcpp::Publisher<MapMsg>::SharedPtr map_publisher_;
  rclcpp::TimerBase::SharedPtr receiver_;

  template <typename... Args>
  static MapDataItr fill_map_lines(MapDataItr itr, int width, int height, Args... args);

  template <typename... Args>
  static MapDataItr fill_map_lines_millor(MapDataItr itr, int width, int height, Args... args);

  template <typename... Args>
  static MapDataItr fill_map_line(MapDataItr itr, int width, int begin, int end, Args... args);

  static MapDataItr fill_map_line(MapDataItr itr, int width);

  template <typename... Args>
  static MapDataItr fill_map_line_millor(MapDataItr itr, int width, int begin, int end, Args... args);

  static MapDataItr fill_map_line_millor(MapDataItr itr, int width);


  void receive();

  void publish_vision_detection(const QByteArray& recv_byte_arr);
};

} // namespace kiks::ssl_vision_bridge

#endif // #ifndef KIKS_SSL_VISION_BRIDGE_BASE_NODE_HPP_
