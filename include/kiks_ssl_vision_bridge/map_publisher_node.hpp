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


#ifndef KIKS_SSL_VISION_BRIDGE__MAP_PUBLISHER_NODE_HPP_
#define KIKS_SSL_VISION_BRIDGE__MAP_PUBLISHER_NODE_HPP_

#include <string>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"

#include "kiks_ssl_vision_bridge/ros_node_base.hpp"
#include "messages_robocup_ssl_geometry.pb.h"

namespace kiks::ssl_vision_bridge
{

class MapPublisherNode : public RosNodeBase
{
public:
  using TimeMsg = builtin_interfaces::msg::Time;

  explicit MapPublisherNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  MapPublisherNode(
    const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  MapPublisherNode(
    const std::string & node_name, const std::string & node_namespace,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  explicit MapPublisherNode(rclcpp::Node::SharedPtr node);

  void publish_map(const TimeMsg & stamp, const SSL_GeometryFieldSize & field);

private:
  using MapMsg = nav_msgs::msg::OccupancyGrid;
  using MapDataItr = std::vector<std::int8_t>::iterator;

  MapMsg map_msg_;
  double map_wall_width_;
  rclcpp::Publisher<MapMsg>::SharedPtr map_publisher_;

  template<typename ... Args>
  static inline MapDataItr fill_map_lines(MapDataItr itr, int width, int height, Args... args);

  template<typename ... Args>
  static inline MapDataItr fill_map_lines_millor(
    MapDataItr itr, int width, int height,
    Args... args);

  template<typename ... Args>
  static inline MapDataItr fill_map_line(
    MapDataItr itr, int width, int begin, int end,
    Args... args);

  static inline MapDataItr fill_map_line(MapDataItr itr, int width);

  template<typename ... Args>
  static inline MapDataItr fill_map_line_millor(
    MapDataItr itr, int width, int begin, int end, Args... args);

  static inline MapDataItr fill_map_line_millor(MapDataItr itr, int width);
};

}  // namespace kiks::ssl_vision_bridge

#endif  // KIKS_SSL_VISION_BRIDGE__MAP_PUBLISHER_NODE_HPP_
