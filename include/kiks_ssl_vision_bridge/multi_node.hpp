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


#ifndef KIKS_SSL_VISION_BRIDGE__MULTI_NODE_HPP_
#define KIKS_SSL_VISION_BRIDGE__MULTI_NODE_HPP_

#include <list>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "kiks_ssl_vision_bridge/ros_node_base.hpp"

namespace kiks::ssl_vision_bridge
{

template<class BaseNode>
class MultiNode : public RosNodeBase
{
public:
  explicit MultiNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : MultiNode(std::make_shared<rclcpp::Node>(BaseNode::default_name(), options))
  {
  }

  MultiNode(
    const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : MultiNode(std::make_shared<rclcpp::Node>(node_name, options))
  {
  }

  MultiNode(
    const std::string & node_name, const std::string & node_namespace,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : MultiNode(std::make_shared<rclcpp::Node>(node_name, options))
  {
  }

  explicit MultiNode(rclcpp::Node::SharedPtr node)
  : RosNodeBase(std::move(node))
  {
    this->add_parameter<std::vector<std::string>>(
      "namespaces", std::vector<std::string>(), [this](const auto & param) {
        for (const auto & sub_namespace : param.as_string_array()) {
          base_node_list_.emplace_back(node_->create_sub_node(sub_namespace));
        }
      });
  }

private:
  std::list<BaseNode> base_node_list_;
};

}  // namespace kiks::ssl_vision_bridge

#endif  // KIKS_SSL_VISION_BRIDGE__MULTI_NODE_HPP_
