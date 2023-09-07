# [kiks_ssl_vision_bridge_ros](../../../README.md)/[reference](../index.md)/class/base_node

#include "kiks_ssl_vision_bridge/ros_node_base.hpp"

kiks::ssl_vision_bridge::BaseNode

#### inheritance
- [ros_node_base](ros_node_base.md)

## public non-member function

#### static std::string default_name()
- Return default node name (="ssl_vision_bridge_base")

## public member function

#### explicit BaseNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
- Initialize with a node created by the default node name (="ssl_vision_bridge_base")

#### BaseNode(const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
- Initialize with a node created by the node name given as an argument

#### BaseNode(const std::string & node_name, const std::string & node_namespace, const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
- Initialize with a node created by the node name and namespace given as arguments

#### explicit BaseNode(rclcpp::Node::SharedPtr node)
- Initialize with a node

###### &copy; 2023 KIKS