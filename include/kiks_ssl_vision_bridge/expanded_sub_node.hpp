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


#ifndef KIKS_SSL_VISION_BRIDGE__EXPANDED_SUB_NODE_HPP_
#define KIKS_SSL_VISION_BRIDGE__EXPANDED_SUB_NODE_HPP_

#include "kiks_ssl_vision_bridge/node_expander.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node.hpp"

namespace kiks::ssl_vision_bridge
{

class ExpandedSubNode : public rclcpp::Node::SharedPtr, public NodeExpander
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(ExpandedSubNode)

  inline ExpandedSubNode(
    const std::string & name,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : ExpandedSubNode(rclcpp::Node::make_shared(name, "/", options))
  {}

  inline ExpandedSubNode(
    const std::string & name, const std::string & ns,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : ExpandedSubNode(rclcpp::Node::make_shared(name, ns, options))
  {}

  explicit inline ExpandedSubNode(rclcpp::Node::SharedPtr node)
  : rclcpp::Node::SharedPtr(std::move(node)),
    NodeExpander(**static_cast<rclcpp::Node::SharedPtr *>(this))
  {}
};

}  // namespace kiks::ssl_vision_bridge

#endif  // KIKS_SSL_VISION_BRIDGE__EXPANDED_SUB_NODE_HPP_
