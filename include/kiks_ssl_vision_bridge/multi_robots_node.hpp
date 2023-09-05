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

#ifndef KIKS_SSL_VISION_BRIDGE__MULTI_ROBOTS_NODE_HPP_
#define KIKS_SSL_VISION_BRIDGE__MULTI_ROBOTS_NODE_HPP_

#include <memory>
#include <string>

#include "kiks_ssl_vision_bridge/multi_node.hpp"
#include "kiks_ssl_vision_bridge/robot_node.hpp"
#include "kiks_ssl_vision_bridge/ros_node_base.hpp"

namespace kiks::ssl_vision_bridge
{

class MultiRobotsNode : public RosNodeBase
{
public:
  static std::string default_name();

  explicit MultiRobotsNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  MultiRobotsNode(
    const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  MultiRobotsNode(
    const std::string & node_name, const std::string & node_namespace,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  explicit MultiRobotsNode(rclcpp::Node::SharedPtr node);

private:
  std::unique_ptr<MultiNode<RobotNode>> multi_node_;
};

}  // namespace kiks::ssl_vision_bridge

#endif  // KIKS_SSL_VISION_BRIDGE__MULTI_ROBOTS_NODE_HPP_
