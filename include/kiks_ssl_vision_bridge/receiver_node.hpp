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


#ifndef KIKS_SSL_VISION_BRIDGE__RECEIVER_NODE_HPP_
#define KIKS_SSL_VISION_BRIDGE__RECEIVER_NODE_HPP_

#include <memory>
#include <string>
#include <unordered_map>

#include "QNetworkInterface"
#include "QUdpSocket"

#include "kiks_ssl_vision_bridge/ball_publisher_sub_node.hpp"
#include "kiks_ssl_vision_bridge/map_publisher_sub_node.hpp"
#include "kiks_ssl_vision_bridge/robot_publisher_sub_node.hpp"
#include "kiks_ssl_vision_bridge/expanded_node.hpp"
#include "messages_robocup_ssl_wrapper.pb.h"

namespace kiks::ssl_vision_bridge
{

class ReceiverNode : public ExpandedNode
{
public:
  static std::string default_name();

  explicit ReceiverNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  ReceiverNode(
    const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  ReceiverNode(
    const std::string & node_name, const std::string & node_namespace,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  explicit ReceiverNode(rclcpp::Node::SharedPtr node);

private:
  using TimeMsg = builtin_interfaces::msg::Time;

  void check_receiving();

  QUdpSocket udp_socket_;
  QHostAddress udp_address_;
  QNetworkInterface udp_interface_;
  std::unordered_map<std::uint32_t, RobotPublisherSubNode> yellow_robot_publisher_sub_nodes_,
    blue_robot_publisher_sub_nodes_;
  std::unique_ptr<BallPublisherSubNode> ball_publisher_sub_node_;
  std::unique_ptr<MapPublisherSubNode> map_publisher_sub_node_;
  rclcpp::TimerBase::SharedPtr receiving_admin_timer_;
};

}  // namespace kiks::ssl_vision_bridge

#endif  // KIKS_SSL_VISION_BRIDGE__RECEIVER_NODE_HPP_
