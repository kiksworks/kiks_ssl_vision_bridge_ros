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


#include "kiks_ssl_vision_bridge/base_node.hpp"

#include <cmath>
#include <cstddef>

#include "QNetworkDatagram"

#include "messages_robocup_ssl_wrapper.pb.h"

namespace kiks::ssl_vision_bridge
{

using namespace std::chrono_literals;

BaseNode::BaseNode(const rclcpp::NodeOptions & options)
: BaseNode(std::make_shared<rclcpp::Node>("ssl_vision_bridge_base", options))
{
}

BaseNode::BaseNode(const std::string & node_name, const rclcpp::NodeOptions & options)
: BaseNode(std::make_shared<rclcpp::Node>(node_name, options))
{
}

BaseNode::BaseNode(
  const std::string & node_name, const std::string & node_namespace,
  const rclcpp::NodeOptions & options)
: BaseNode(std::make_shared<rclcpp::Node>(node_name, node_namespace, options))
{
}

BaseNode::BaseNode(rclcpp::Node::SharedPtr node)
: RosNodeBase(std::move(node))
{
  this->add_parameter<std::int64_t>(
    "udp.port", 10006, [this](const auto & param) {
      udp_socket_.bind(QHostAddress::AnyIPv4, param.as_int());
    });
  this->add_parameter<std::string>(
    "udp.address", "224.5.23.2", [this](const auto & param) {
      udp_socket_.joinMulticastGroup(QHostAddress(param.as_string().c_str()));
    });
  this->add_parameter<bool>(
    "map.enable", true, [this](const auto & param) {
      if (param.as_bool()) {
        map_publisher_ = node_->create_publisher<MapMsg>("map", rclcpp::QoS(4));
      } else {
        map_publisher_.reset();
      }
    });
  this->add_parameter<double>(
    "map.resolution", 0.01, [this](const auto & param) {
      map_msg_.info.resolution = param.as_double();
    });
  this->add_parameter<double>(
    "map.wall_width", 0.1, [this](const auto & param) {map_wall_width_ = param.as_double();});
  this->add_parameter<std::string>(
    "map.frame_id", "map", [this](const auto & param) {
      map_msg_.header.frame_id = param.as_string();
    });

  vision_detection_publisher_ =
    node_->create_publisher<VisionDetectionMsg>("vision_detection", rclcpp::QoS(4).best_effort());

  timer_of_recv_ = node_->create_wall_timer(10ms, std::bind(&BaseNode::receive, this));
}

void BaseNode::receive()
{
  auto now = node_->now();
  const auto timeout_time = now + 20ms;
  auto end_time = timeout_time;

  bool timer_has_reseted = false;
  auto reset_timer_once = [&timer_has_reseted, this]() {
      if (!timer_has_reseted) {
        timer_of_recv_->reset();
        timer_has_reseted = true;
      }
    };

  do {
    if (udp_socket_.hasPendingDatagrams()) {
      this->publish_vision_detection(udp_socket_.receiveDatagram().data());
      reset_timer_once();
      end_time = std::min(now + 1ms, timeout_time);
    }
    now = node_->now();
  } while (now < end_time);

  reset_timer_once();
}

template<typename ... Args>
BaseNode::MapDataItr BaseNode::fill_map_lines(MapDataItr itr, int width, int height, Args... args)
{
  auto itr_end = itr + height * width;
  while (itr < itr_end) {
    itr = fill_map_line(itr, width, args ...);
  }
  return itr_end;
}

template<typename ... Args>
BaseNode::MapDataItr BaseNode::fill_map_lines_millor(
  MapDataItr itr, int width, int height, Args... args)
{
  auto itr_end = itr + height * width;
  while (itr < itr_end) {
    itr = fill_map_line_millor(itr, width, args ...);
  }
  return itr_end;
}

template<typename ... Args>
BaseNode::MapDataItr BaseNode::fill_map_line(
  MapDataItr itr, int width, int begin, int end, Args... args)
{
  std::for_each(itr + begin, itr + end, [](auto & val) {val = 100;});
  return fill_map_line(itr, width, args ...);
}

BaseNode::MapDataItr BaseNode::fill_map_line(MapDataItr itr, int width) {return itr + width;}

template<typename ... Args>
BaseNode::MapDataItr BaseNode::fill_map_line_millor(
  MapDataItr itr, int width, int begin, int end, Args... args)
{
  std::for_each(itr + begin, itr + end, [](auto & val) {val = 100;});
  std::for_each(itr + width - end, itr + width - begin, [](auto & val) {val = 100;});
  return fill_map_line_millor(itr, width, args ...);
}

BaseNode::MapDataItr BaseNode::fill_map_line_millor(MapDataItr itr, int width)
{
  return itr + width;
}

void BaseNode::publish_vision_detection(const QByteArray & recv_byte_arr)
{
  thread_local SSL_WrapperPacket packet;
  if (!packet.ParseFromArray(recv_byte_arr.data(), recv_byte_arr.size())) {
    RCLCPP_WARN(node_->get_logger(), "cannnot parse buffer");
    return;
  }

  auto now = node_->now();

  if (packet.has_detection()) {
    const auto & detection = packet.detection();

    auto vision_detection_msg = std::make_unique<VisionDetectionMsg>();
    vision_detection_msg->header.stamp = now;
    vision_detection_msg->header.frame_id = map_msg_.header.frame_id;

    for (const auto & ball : detection.balls()) {
      PointMsg ball_msg;
      ball_msg.x = ball.x() * 0.001;
      ball_msg.y = ball.y() * 0.001;
      ball_msg.z = ball.z() * 0.001;
      vision_detection_msg->balls.push_back(ball_msg);
    }

    auto convert_robots = [](const auto & robots, auto & robots_msg) {
        for (const auto & robot : robots) {
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

    convert_robots(detection.robots_yellow(), vision_detection_msg->yellow_robots);
    convert_robots(detection.robots_blue(), vision_detection_msg->blue_robots);

    vision_detection_publisher_->publish(std::move(vision_detection_msg));
  }

  if (map_publisher_ && packet.has_geometry()) {
    map_msg_.header.stamp = now;
    const auto & field = packet.geometry().field();
    map_msg_.info.map_load_time = now;
    auto width =
      field.field_length() * 0.001 + (field.boundary_width() * 0.001 + map_wall_width_) * 2;
    auto height =
      field.field_width() * 0.001 + (field.boundary_width() * 0.001 + map_wall_width_) * 2;
    map_msg_.info.width = width / map_msg_.info.resolution;
    map_msg_.info.height = height / map_msg_.info.resolution;
    int wall_count = map_wall_width_ / map_msg_.info.resolution;
    int boundary_width = field.boundary_width() * 0.001 / map_msg_.info.resolution;
    int goal_width = field.goal_depth() * 0.001 / map_msg_.info.resolution;
    int goal_height = field.goal_width() * 0.001 / map_msg_.info.resolution;
    int wall_to_goal_height = (map_msg_.info.height - goal_height) * 0.5 - wall_count * 2;
    map_msg_.info.origin.position.x = width * -0.5;
    map_msg_.info.origin.position.y = height * -0.5;
    map_msg_.info.origin.orientation.w = 1;
    map_msg_.data.resize(map_msg_.info.width * map_msg_.info.height);
    auto itr = map_msg_.data.begin();
    itr = fill_map_lines(itr, map_msg_.info.width, wall_count, 0, map_msg_.info.width);
    itr = fill_map_lines_millor(itr, map_msg_.info.width, wall_to_goal_height, 0, wall_count);
    itr =
      fill_map_lines_millor(itr, map_msg_.info.width, wall_count, 0, wall_count + boundary_width);
    itr = fill_map_lines_millor(
      itr, map_msg_.info.width, goal_height, 0, wall_count + boundary_width - goal_width);
    itr =
      fill_map_lines_millor(itr, map_msg_.info.width, wall_count, 0, wall_count + boundary_width);
    itr = fill_map_lines_millor(itr, map_msg_.info.width, wall_to_goal_height, 0, wall_count);
    itr = fill_map_lines(itr, map_msg_.info.width, wall_count, 0, map_msg_.info.width);
    map_publisher_->publish(map_msg_);
  }
}

}  // namespace kiks::ssl_vision_bridge
