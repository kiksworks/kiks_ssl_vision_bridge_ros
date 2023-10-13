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
: ReceiverNode("ssl_vision_bridge", "", options)
{
}

ReceiverNode::ReceiverNode(const std::string & node_name, const rclcpp::NodeOptions & options)
: ReceiverNode(node_name, "", options)
{
}

ReceiverNode::ReceiverNode(
  const std::string & node_name, const std::string & node_namespace,
  const rclcpp::NodeOptions & options)
: ExpandedNode(node_name, node_namespace, options)
{
  // Parameter of ssl-vision udp port
  this->add_param<std::int64_t>(
    "udp.port", 10006, [this](const auto & param) {
      udp_socket_.bind(QHostAddress::AnyIPv4, param.as_int());
      udp_socket_.joinMulticastGroup(udp_address_, udp_interface_);
    });
  // Parameter of ssl-vision udp multicast ip
  this->add_param<std::string>(
    "udp.address", "224.5.23.2", [this](const auto & param) {
      udp_address_ = QHostAddress(param.as_string().c_str());
      udp_socket_.joinMulticastGroup(udp_address_);
    });
  // Parameter of ssl-vision udp multicast interface
  this->add_param<std::string>(
    "udp.interface", "", [this](const auto & param) {
      udp_interface_ = QNetworkInterface::interfaceFromName(param.as_string().c_str());
      if (param.as_string() != "" && !udp_interface_.isValid()) {
        RCLCPP_WARN(
          this->get_logger(),
          "\"%s\" is an invalid interface (use the default interface instead).",
          param.as_string().c_str());
      }
      udp_socket_.setMulticastInterface(udp_interface_);
    });
  // Parameter of each team robots
  // Default is {"teamcolor0, teamcolor1 ... "teamcolor14", "teamcolor15"} for each team
  auto create_robots_str = [this](const std::string & base_name) {
      std::vector<std::string> namespaces;
      for (int i = 0; i < 16; ++i) {
        const auto ns =
          (i < 10) ? (base_name + "0" + std::to_string(i)) : (base_name + std::to_string(i));
        namespaces.push_back(ns);
      }
      return namespaces;
    };
  auto set_robots = [this](auto & robots, const std::vector<std::string> str_arr) {
      robots.clear();
      for (std::uint32_t i = 0; i < str_arr.size(); ++i) {
        const auto & str = str_arr[i];
        if (str == "/") {
          robots.emplace(i, this->shared_from_this());
        }
        else if(str != "") {
          robots.emplace(i, this->create_sub_node(str));
        }
      }
    };
  this->add_param<std::vector<std::string>>(
    "yellow_robots", create_robots_str("yellow"), [this, set_robots](const auto & param) {
      set_robots(yellow_robot_publisher_sub_nodes_, param.as_string_array());
    });
  this->add_param<std::vector<std::string>>(
    "blue_robots", create_robots_str("blue"), [this, set_robots](const auto & param) {
      set_robots(blue_robot_publisher_sub_nodes_, param.as_string_array());
    });
  // Parameter of ball publisher enable
  this->add_param<bool>(
    "ball.enable", true, [this](const auto & param) {
      ball_publisher_sub_node_ = param.as_bool() ?
      std::make_unique<BallPublisherSubNode>(this->create_sub_node("ball")) :
      std::unique_ptr<BallPublisherSubNode>();
    });
  // Parameter of map publisher enable
  this->add_param<bool>(
    "map.enable", true, [this](const auto & param) {
      map_publisher_sub_node_ = param.as_bool() ?
      std::make_unique<MapPublisherSubNode>(this->create_sub_node("map")) :
      std::unique_ptr<MapPublisherSubNode>();
    });
  // Set timer to check reception
  receiving_admin_timer_ =
    this->create_wall_timer(10ms, std::bind(&ReceiverNode::check_receiving, this));
}

void ReceiverNode::check_receiving()
{
  auto now = this->now();
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
    now = this->now();
    // Check receiving
    if (!udp_socket_.hasPendingDatagrams()) {
      continue;
    }

    // Parse to packet
    const auto recv_byte_arr = udp_socket_.receiveDatagram().data();
    thread_local SSL_WrapperPacket packet;
    if (!packet.ParseFromArray(recv_byte_arr.data(), recv_byte_arr.size())) {
      RCLCPP_WARN(this->get_logger(), "cannnot parse buffer");
      continue;
    }
    // Publish detection
    if (packet.has_detection()) {
      const auto & detection = packet.detection();
      // Time stamp calculation.
      // The time taken for ssl-vision processing is added to the current time.
      const std::uint64_t delay_ns = (detection.t_sent() - detection.t_capture()) *
        (0.001 * 0.001 * 0.001);
      const TimeMsg stamp = now - std::chrono::nanoseconds(delay_ns);
      // Run all robot publishers
      const auto publish_robots = [&stamp](auto & nodes, const auto & robots) {
          for (const auto & robot : robots) {
            auto itr = nodes.find(robot.robot_id());
            if (itr == nodes.end()) {
              continue;
            }
            itr->second.publish_robot(stamp, robot);
          }
        };
      publish_robots(yellow_robot_publisher_sub_nodes_, detection.robots_yellow());
      publish_robots(blue_robot_publisher_sub_nodes_, detection.robots_blue());
      // Run ball publisher
      if (ball_publisher_sub_node_ && detection.balls().size() >= 1) {
        ball_publisher_sub_node_->publish_ball(stamp, detection.balls(0));
      }
    }
    // Publish geometry
    if (map_publisher_sub_node_ && packet.has_geometry()) {
      map_publisher_sub_node_->publish_map(now, packet.geometry().field());
    }
    // Reset timer
    reset_timer_once();
    end_time = std::min(now + 1ms, timeout_time);
  } while (now < end_time);

  reset_timer_once();
}

}  // namespace kiks::ssl_vision_bridge
