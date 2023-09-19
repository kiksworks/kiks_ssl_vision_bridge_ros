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


#include "kiks_ssl_vision_bridge/receiver_node.hpp"

#include <cstdint>

#include "QNetworkDatagram"

#include "messages_robocup_ssl_wrapper.pb.h"

namespace kiks::ssl_vision_bridge
{

using namespace std::chrono_literals;

ReceiverNode::ReceiverNode(const rclcpp::NodeOptions & options)
: ReceiverNode(std::make_shared<rclcpp::Node>("ssl_vision_bridge", options))
{
}

ReceiverNode::ReceiverNode(const std::string & node_name, const rclcpp::NodeOptions & options)
: ReceiverNode(std::make_shared<rclcpp::Node>(node_name, options))
{
}

ReceiverNode::ReceiverNode(
  const std::string & node_name, const std::string & node_namespace,
  const rclcpp::NodeOptions & options)
: ReceiverNode(std::make_shared<rclcpp::Node>(node_name, node_namespace, options))
{
}

ReceiverNode::ReceiverNode(rclcpp::Node::SharedPtr node)
: RosNodeBase(std::move(node))
{
  // Parameter of ssl-vision udp port
  this->add_parameter<std::int64_t>(
    "udp.port", 10006, [this](const auto & param) {
      udp_socket_.bind(QHostAddress::AnyIPv4, param.as_int());
    });
  // Parameter of ssl-vision udp multicast ip
  this->add_parameter<std::string>(
    "udp.address", "224.5.23.2", [this](const auto & param) {
      udp_socket_.joinMulticastGroup(QHostAddress(param.as_string().c_str()));
    });
  
  // Parameter of each team robots
  // Default is {"teamcolor0, teamcolor1 ... "teamcolor14", "teamcolor15"} for each team
  auto create_robots = [this](const std::string& base_name) {
      std::vector<std::string> namespaces;
      for (int i = 0; i < 16; ++i) {
        auto ns = base_name + std::to_string(i);
        namespaces.push_back(ns);
        this->add_parameter<std::int64_t>(ns + ".robot_id", i, [](const auto &) {});
      }
      return namespaces;
    };
  this->add_parameter<std::vector<std::string>>("yellow_robots", create_robots("yellow"), [this](const auto & param) {
    yellow_robot_publisher_nodes_.clear();
    for(auto str : param.as_string_array()) {
      yellow_robot_publisher_nodes_.emplace_back(node_->create_sub_node(str));
    }
  });
  this->add_parameter<std::vector<std::string>>("blue_robots", create_robots("blue"), [this](const auto & param) {
    blue_robot_publisher_nodes_.clear();
    for(auto str : param.as_string_array()) {
      blue_robot_publisher_nodes_.emplace_back(node_->create_sub_node(str));
    }
  });
  // Parameter of ball publisher enable
  this->add_parameter<bool>(
    "ball.enable", true, [this](const auto & param) {
      ball_publisher_node_ = param.as_bool() ?
        std::make_unique<BallPublisherNode>(node_->create_sub_node("ball")) :
        std::unique_ptr<BallPublisherNode>();
    });
  // Parameter of map publisher enable
  this->add_parameter<bool>(
    "map.enable", true, [this](const auto & param) {
      map_publisher_node_ = param.as_bool() ?
        std::make_unique<MapPublisherNode>(node_->create_sub_node("map")) :
        std::unique_ptr<MapPublisherNode>();
    });
  // Set timer to check reception
  receiving_admin_timer_ = node_->create_wall_timer(10ms, std::bind(&ReceiverNode::check_receiving, this));
}

void ReceiverNode::check_receiving()
{
  auto now = node_->now();
  const auto timeout_time = now + 20ms;
  auto end_time = timeout_time;

  // Set timer resetter
  // - This function resets the timer only once with this call.
  bool timer_has_reseted = false;
  auto reset_timer_once = [&timer_has_reseted, this]() {
      if (!timer_has_reseted) {
        receiving_admin_timer_->reset();
        timer_has_reseted = true;
      }
    };

  do {
    now = node_->now();
    // Check receiving
    if (!udp_socket_.hasPendingDatagrams()) {
      continue;
    }

    // Parse to packet
    const auto recv_byte_arr = udp_socket_.receiveDatagram().data();
    thread_local SSL_WrapperPacket packet;
    if (!packet.ParseFromArray(recv_byte_arr.data(), recv_byte_arr.size())) {
      RCLCPP_WARN(node_->get_logger(), "cannnot parse buffer");
      continue;
    }
    // Publish detection
    if(packet.has_detection()) {
      const auto& detection = packet.detection();
      // Time stamp calculation.
      // The time taken for ssl-vision processing is added to the current time.
      const std::uint64_t delay_ns = (detection.t_sent() - detection.t_capture()) * (0.001 * 0.001 * 0.001);
      const TimeMsg stamp = now + std::chrono::nanoseconds(delay_ns);
      // Run all robot publishers
      const auto publish_robots = [&stamp](auto& nodes, const auto& robots) {
          for(const auto& robot : robots) {
            for(auto& node : nodes) {
              node.publish_robot(stamp, robot);
            }
          }
        };
      publish_robots(yellow_robot_publisher_nodes_, detection.robots_yellow());
      publish_robots(blue_robot_publisher_nodes_, detection.robots_blue());
      // Run ball publisher
      if(ball_publisher_node_ && detection.balls().size() >= 1) {
        ball_publisher_node_->publish_ball(stamp, detection.balls(0));
      }
    }
    // Publish geometry
    if(map_publisher_node_ && packet.has_geometry()) {
      map_publisher_node_->publish_map(now, packet.geometry().field());
    }
    // Reset timer
    reset_timer_once();
    end_time = std::min(now + 1ms, timeout_time);
  } while (now < end_time);

  reset_timer_once();
}

}  // namespace kiks::ssl_vision_bridge
