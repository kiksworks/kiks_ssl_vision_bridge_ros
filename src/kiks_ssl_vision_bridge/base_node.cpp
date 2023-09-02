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

#include "kiks_ssl_vision_bridge/base_node.hpp"

#include <cmath>

#include "QNetworkDatagram"

#include "messages_robocup_ssl_wrapper.pb.h"

namespace kiks::ssl_vision_bridge
{

using namespace std::chrono_literals;

BaseNode::BaseNode(const rclcpp::NodeOptions &options) :
  BaseNode(std::make_shared<rclcpp::Node>("ssl_vision_bridge_base", options)) {}

BaseNode::BaseNode(
  const std::string &node_name,
  const rclcpp::NodeOptions &options) :
  BaseNode(std::make_shared<rclcpp::Node>(node_name, options)) {}

BaseNode::BaseNode(
  const std::string &node_name,
  const std::string &node_namespace, 
  const rclcpp::NodeOptions &options) :
  BaseNode(std::make_shared<rclcpp::Node>(node_name, node_namespace, options)) {}
  
BaseNode::BaseNode(rclcpp::Node::SharedPtr node) :
  node_(std::move(node)),
  map_frame_id_(node_->declare_parameter<std::string>("map.frame_id", "map")),
  vision_detection_publisher_(node_->create_publisher<VisionDetectionMsg>("vision_detection", rclcpp::QoS(4).best_effort())),
  receiver_(node_->create_wall_timer(10ms, std::bind(&BaseNode::receive, this)))
  {
    udp_socket_.bind(QHostAddress::AnyIPv4, 10009);
    udp_socket_.joinMulticastGroup(QHostAddress("224.5.23.2"));
    udp_socket_.abort();
    udp_socket_.bind(QHostAddress::AnyIPv4, 10006);
    udp_socket_.joinMulticastGroup(QHostAddress("224.5.23.2"));
  }

void BaseNode::receive()
{
  auto now = node_->now();
  const auto timeout_time = now + 20ms;
  auto end_time = timeout_time;

  bool timer_has_reseted = false;
  auto reset_timer_once = [&timer_has_reseted, this](){
    if(!timer_has_reseted) {
      receiver_->reset();
      timer_has_reseted = true;
    }
  };

  do {
    if(udp_socket_.hasPendingDatagrams()) {
      this->publish_vision_detection(udp_socket_.receiveDatagram().data());
      reset_timer_once();
      end_time = std::min(now + 1ms, timeout_time);
    }
    now = node_->now();
  } while(now < end_time);

  reset_timer_once();
}

void BaseNode::publish_vision_detection(const QByteArray& recv_byte_arr)
{
  thread_local SSL_WrapperPacket packet;
  if (!packet.ParseFromArray(recv_byte_arr.data(), recv_byte_arr.size())) {
    RCLCPP_WARN(node_->get_logger(), "cannnot parse buffer");
    return;
  }

  if(packet.has_detection()) {
    const auto & detection = packet.detection();

    VisionDetectionMsg vision_detection_msg;
    vision_detection_msg.header.stamp = node_->now();
    vision_detection_msg.header.frame_id = map_frame_id_;

    for (const auto & ball : detection.balls()) {
      PointMsg ball_msg;
      ball_msg.x = ball.x() * 0.001;
      ball_msg.y = ball.y() * 0.001;
      ball_msg.z = ball.z() * 0.001;
      vision_detection_msg.balls.push_back(ball_msg);
    }
    
    auto convert_robots = [](const auto& robots, auto& robots_msg) {
      for(const auto& robot : robots) {
        PoseWithIdMsg robot_msg;
        robot_msg.id = robot.robot_id();
        robot_msg.pose.position.x = robot.x() * 0.001;
        robot_msg.pose.position.y = robot.y() * 0.001;
        auto theta = robot.orientation() * 0.5;
        robot_msg.pose.orientation.z = std::sin(theta);
        robot_msg.pose.orientation.w = std::cos(theta);
        robots_msg.push_back(robot_msg);
      }
    };

    convert_robots(detection.robots_yellow(), vision_detection_msg.yellow_robots);
    convert_robots(detection.robots_blue(), vision_detection_msg.blue_robots);

    vision_detection_publisher_->publish(vision_detection_msg);
  }

  if(packet.has_geometry()) {
    
  }
}

} // namespace kiks::ssl_vision_bridge
