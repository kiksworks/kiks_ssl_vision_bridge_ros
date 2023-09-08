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


#include "kiks_ssl_vision_bridge/multi_robots_node.hpp"

#include <cstddef>

namespace kiks::ssl_vision_bridge
{

using namespace std::chrono_literals;

std::string MultiRobotsNode::default_name() {return "ssl_vision_bridge_multi_robots";}

MultiRobotsNode::MultiRobotsNode(const rclcpp::NodeOptions & options)
: MultiRobotsNode(std::make_shared<rclcpp::Node>(default_name(), options))
{
}

MultiRobotsNode::MultiRobotsNode(const std::string & node_name, const rclcpp::NodeOptions & options)
: MultiRobotsNode(std::make_shared<rclcpp::Node>(node_name, options))
{
}

MultiRobotsNode::MultiRobotsNode(
  const std::string & node_name, const std::string & node_namespace,
  const rclcpp::NodeOptions & options)
: MultiRobotsNode(std::make_shared<rclcpp::Node>(node_name, node_namespace, options))
{
}

MultiRobotsNode::MultiRobotsNode(rclcpp::Node::SharedPtr node)
: RosNodeBase(std::move(node))
{
  std::vector<std::string> namespaces;
  for (int i = 0; i < 11; ++i) {
    auto add_namespace_and_param = [this, &namespaces, i](std::string color) {
        auto ns = color + std::to_string(i);
        namespaces.push_back(ns);
        this->add_parameter<std::string>(ns + ".team_color", color, [](const auto &) {});
        this->add_parameter<std::int64_t>(ns + ".robot_id", i, [](const auto &) {});
      };
    add_namespace_and_param("yellow");
    add_namespace_and_param("blue");
  }

  this->add_parameter<std::vector<std::string>>("namespaces", namespaces, [](const auto &) {});

  multi_node_ = std::make_unique<MultiNode<RobotNode>>(node_);
}

}  // namespace kiks::ssl_vision_bridge
