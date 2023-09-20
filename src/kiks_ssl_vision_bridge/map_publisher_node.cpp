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


#include "kiks_ssl_vision_bridge/map_publisher_node.hpp"

#include <cmath>

namespace kiks::ssl_vision_bridge
{

MapPublisherNode::MapPublisherNode(const rclcpp::NodeOptions & options)
: MapPublisherNode(std::make_shared<rclcpp::Node>("ssl_vision_bridge_map_publisher", options))
{
}

MapPublisherNode::MapPublisherNode(
  const std::string & node_name,
  const rclcpp::NodeOptions & options)
: MapPublisherNode(std::make_shared<rclcpp::Node>(node_name, options))
{
}

MapPublisherNode::MapPublisherNode(
  const std::string & node_name, const std::string & node_namespace,
  const rclcpp::NodeOptions & options)
: MapPublisherNode(std::make_shared<rclcpp::Node>(node_name, node_namespace, options))
{
}

MapPublisherNode::MapPublisherNode(rclcpp::Node::SharedPtr node)
: RosNodeBase(std::move(node))
{
  // Parameter of map resolution
  this->add_parameter<double>(
    "map.resolution", 0.01, [this](const auto & param) {
      map_msg_.info.resolution = param.as_double();
    });
  // Parameter of field and goal thickness
  this->add_parameter<double>(
    "map.wall_width", 0.02, [this](const auto & param) {map_wall_width_ = param.as_double();});
  // Parameter of map frame_id
  const std::string ns = node_->get_namespace();
  const std::string ns_with_slash = ns.back() == '/' ? ns : ns + '/';
  this->add_parameter<std::string>(
    "map.frame_id", "map", [this, ns_with_slash](const auto & param) {
      const auto str = param.as_string();
      map_msg_.header.frame_id = str;
      map_publisher_ = node_->create_publisher<MapMsg>(
        ns_with_slash + str,
        this->get_dynamic_qos());
    });
}

void MapPublisherNode::publish_map(const TimeMsg & stamp, const SSL_GeometryFieldSize & field)
{
  map_msg_.header.stamp = stamp;
  map_msg_.info.map_load_time = stamp;
  auto width =
    field.field_length() * 0.001 + (field.boundary_width() * 0.001 + map_wall_width_) * 2;
  auto height =
    field.field_width() * 0.001 + (field.boundary_width() * 0.001 + map_wall_width_) * 2;
  map_msg_.info.width = width / map_msg_.info.resolution;
  map_msg_.info.height = height / map_msg_.info.resolution;
  int wall_count = map_wall_width_ / map_msg_.info.resolution;
  int boundary_width = field.boundary_width() * 0.001 / map_msg_.info.resolution;
  int goal_width = field.goal_depth() * 0.001 / map_msg_.info.resolution;
  int goal_height = field.goal_width() * 0.001 / map_msg_.info.resolution;
  int wall_to_goal_height = (map_msg_.info.height - goal_height) * 0.5 - wall_count * 2;
  map_msg_.info.origin.position.x = width * -0.5;
  map_msg_.info.origin.position.y = height * -0.5;
  map_msg_.info.origin.orientation.w = 1;
  map_msg_.data.resize(map_msg_.info.width * map_msg_.info.height);
  auto itr = map_msg_.data.begin();
  itr = fill_map_lines(itr, map_msg_.info.width, wall_count, 0, map_msg_.info.width);
  itr = fill_map_lines_millor(itr, map_msg_.info.width, wall_to_goal_height, 0, wall_count);
  itr =
    fill_map_lines_millor(itr, map_msg_.info.width, wall_count, 0, wall_count + boundary_width);
  itr = fill_map_lines_millor(
    itr, map_msg_.info.width, goal_height, 0, wall_count + boundary_width - goal_width);
  itr =
    fill_map_lines_millor(itr, map_msg_.info.width, wall_count, 0, wall_count + boundary_width);
  itr = fill_map_lines_millor(itr, map_msg_.info.width, wall_to_goal_height, 0, wall_count);
  itr = fill_map_lines(itr, map_msg_.info.width, wall_count, 0, map_msg_.info.width);
  map_publisher_->publish(map_msg_);
}

template<typename ... Args>
MapPublisherNode::MapDataItr MapPublisherNode::fill_map_lines(
  MapDataItr itr, int width, int height,
  Args... args)
{
  auto itr_end = itr + height * width;
  while (itr < itr_end) {
    itr = fill_map_line(itr, width, args ...);
  }
  return itr_end;
}

template<typename ... Args>
MapPublisherNode::MapDataItr MapPublisherNode::fill_map_lines_millor(
  MapDataItr itr, int width, int height, Args... args)
{
  auto itr_end = itr + height * width;
  while (itr < itr_end) {
    itr = fill_map_line_millor(itr, width, args ...);
  }
  return itr_end;
}

template<typename ... Args>
MapPublisherNode::MapDataItr MapPublisherNode::fill_map_line(
  MapDataItr itr, int width, int begin, int end, Args... args)
{
  std::for_each(itr + begin, itr + end, [](auto & val) {val = 100;});
  return fill_map_line(itr, width, args ...);
}

MapPublisherNode::MapDataItr MapPublisherNode::fill_map_line(MapDataItr itr, int width)
{
  return itr + width;
}

template<typename ... Args>
MapPublisherNode::MapDataItr MapPublisherNode::fill_map_line_millor(
  MapDataItr itr, int width, int begin, int end, Args... args)
{
  std::for_each(itr + begin, itr + end, [](auto & val) {val = 100;});
  std::for_each(itr + width - end, itr + width - begin, [](auto & val) {val = 100;});
  return fill_map_line_millor(itr, width, args ...);
}

MapPublisherNode::MapDataItr MapPublisherNode::fill_map_line_millor(MapDataItr itr, int width)
{
  return itr + width;
}

}  // namespace kiks::ssl_vision_bridge
