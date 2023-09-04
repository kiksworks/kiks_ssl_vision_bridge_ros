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

#ifndef KIKS_SSL_VISION_BRIDGE_ROS_NODE_BASE_HPP_
#define KIKS_SSL_VISION_BRIDGE_ROS_NODE_BASE_HPP_

#include <memory>
#include <string>
#include <unordered_map>

#include "rclcpp/node.hpp"

namespace kiks::ssl_vision_bridge
{

class RosNodeBase
{
public:
  RosNodeBase(rclcpp::Node::SharedPtr node);

  operator rclcpp::Node::SharedPtr() {
    return node_;
  }

protected:
  using SetParamFunc = std::function<void(const rclcpp::Parameter &)>;

  template <typename T>
  void add_parameter(std::string name, T default_value, SetParamFunc set_param_func);

  template <typename T, typename MemberFunc, typename ClassPtr>
  void add_parameter(std::string name, T default_value, MemberFunc member_func, ClassPtr class_ptr) {
    create_parameter(std::move(name), default_value, std::bind(member_func, class_ptr, std::placeholders::_1));
  }

  rclcpp::Node::SharedPtr node_;

  std::unordered_map<std::string, SetParamFunc> set_param_func_map_;

private:
  rcl_interfaces::msg::SetParametersResult set_parameters(std::vector<rclcpp::Parameter> parameters);

#ifdef KIKS_ROS_DISTRO_DASHING
  OnParametersSetCallbackType on_set_parameters_callback_handle_;
#else
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;
#endif
};

} // namespace kiks::ssl_vision_bridge

#endif // #ifndef KIKS_SSL_VISION_BRIDGE_ROS_NODE_BASE_HPP_
