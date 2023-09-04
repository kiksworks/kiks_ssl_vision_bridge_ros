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

#ifndef KIKS_SSL_VISION_BRIDGE_MULTI_NODE_HPP_
#define KIKS_SSL_VISION_BRIDGE_MULTI_NODE_HPP_

#include <list>

#include "kiks_ssl_vision_bridge/ros_node_base.hpp"

namespace kiks::ssl_vision_bridge
{

template <class BaseNode>
class MultiNode : public RosNodeBase
{
public:
  MultiNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) :
    MultiNode(std::make_shared<rclcpp::Node>(BaseNode::default_name(), options)) {}
  
  MultiNode(
    const std::string &node_name,
    const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) :
    MultiNode(std::make_shared<rclcpp::Node>(node_name, options)) {}

  MultiNode(
    const std::string &node_name,
    const std::string &node_namespace,
    const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) :
    MultiNode(std::make_shared<rclcpp::Node>(node_name, options)) {}
    
  MultiNode(rclcpp::Node::SharedPtr node) :
    RosNodeBase(std::move(node))
  {
    this->add_parameter<std::vector<std::string>>("namespaces", std::vector<std::string>(), [this](const auto& param){
      for(const auto& sub_namespace : param.as_string_array()) {
        base_node_list_.emplace_back(node_->create_sub_node(sub_namespace));
      }
    });
  }

private:
  std::list<BaseNode> base_node_list_;
};

} // namespace kiks::ssl_vision_bridge

#endif // #ifndef KIKS_SSL_VISION_BRIDGE_MULTI_NODE_HPP_
