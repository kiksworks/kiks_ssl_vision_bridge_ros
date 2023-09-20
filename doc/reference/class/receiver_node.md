# [kiks_ssl_vision_bridge_ros](../../../README.md)/[reference](../index.md)/class/receiver_node

#include "kiks_ssl_vision_bridge/receiver_node.hpp"

kiks::ssl_vision_bridge::ReceiverNode

#### inheritance
- [ros_node_base](ros_node_base.md)

## Description
- TReceives UDP data from ssl-vision and publishes it by member pulisher nodes.
- It has multiple [robot_publisher_node](robot_publisher_node.md)s and one [ball_publisher_node](ball_publisher_node.md) and [map_publisher_node](map_publisher_node.md) as members.

## public non-member function

#### static std::string default_name()
- Return default node name (="ssl_vision_bridge")

## public member function

#### explicit ReceiverNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
- Initialize with a node created by the default node name (="ssl_vision_bridge")

#### ReceiverNode(const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
- Initialize with a node created by the node name given as an argument

#### ReceiverNode(const std::string & node_name, const std::string & node_namespace, const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
- Initialize with a node created by the node name and namespace given as arguments

#### explicit ReceiverNode(rclcpp::Node::SharedPtr node)
- Initialize with a node

## Related

### class
- [robot_publisher_node](robot_publisher_node.md)
- [ball_publisher_node](ball_publisher_node.md)
- [map_publisher_node](map_publisher_node.md)

### node
- [receiver](../node/receiver.md)

###### &copy; 2023 KIKS