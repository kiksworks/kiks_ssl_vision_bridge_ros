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


#ifndef KIKS_SSL_VISION_BRIDGE__BASE_NODE_HPP_
#define KIKS_SSL_VISION_BRIDGE__BASE_NODE_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "QUdpSocket"

#include "kiks_ssl_vision_bridge/msg/vision_detection.hpp"
#include "kiks_ssl_vision_bridge/ros_node_base.hpp"

namespace kiks::ssl_vision_bridge
{

class BaseNode : public RosNodeBase
{
public:
  explicit BaseNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  BaseNode(
    const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  BaseNode(
    const std::string & node_name, const std::string & node_namespace,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  explicit BaseNode(rclcpp::Node::SharedPtr node);

private:
  using VisionDetectionMsg = kiks_ssl_vision_bridge::msg::VisionDetection;
  using PoseWithIdMsg = kiks_ssl_vision_bridge::msg::PoseWithId;
  using PointMsg = geometry_msgs::msg::Point;
  using MapMsg = nav_msgs::msg::OccupancyGrid;
  using MapDataItr = std::vector<std::int8_t>::iterator;

  QUdpSocket udp_socket_;
  std::string map_frame_id_;
  rclcpp::Publisher<VisionDetectionMsg>::SharedPtr vision_detection_publisher_;
  bool map_enable_;
  MapMsg map_msg_;
  double map_wall_width_;
  rclcpp::Publisher<MapMsg>::SharedPtr map_publisher_;
  rclcpp::TimerBase::SharedPtr timer_of_recv_;

  template<typename ... Args>
  static MapDataItr fill_map_lines(MapDataItr itr, int width, int height, Args... args);

  template<typename ... Args>
  static MapDataItr fill_map_lines_millor(MapDataItr itr, int width, int height, Args... args);

  template<typename ... Args>
  static MapDataItr fill_map_line(MapDataItr itr, int width, int begin, int end, Args... args);

  static MapDataItr fill_map_line(MapDataItr itr, int width);

  template<typename ... Args>
  static MapDataItr fill_map_line_millor(
    MapDataItr itr, int width, int begin, int end, Args... args);

  static MapDataItr fill_map_line_millor(MapDataItr itr, int width);

  void receive();

  inline void publish_vision_detection(const QByteArray & recv_byte_arr);
};

}  // namespace kiks::ssl_vision_bridge

#endif  // KIKS_SSL_VISION_BRIDGE__BASE_NODE_HPP_
