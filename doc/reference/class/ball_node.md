# [kiks_ssl_vision_bridge_ros](../../../README.md)/[reference](../index.md)/class/ball_node

#include "kiks_ssl_vision_bridge/ball_node.hpp"

kiks::ssl_vision_bridge::BallNode

#### inheritance
- [ros_node_base](ros_node_base.md)

## public non-member function

#### static std::string default_name()
- Return default node name (="ssl_bridge_vision_ball")

## public member function

#### explicit BallNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
- Initialize with a node created by the default node name (="ssl_bridge_vision_ball")

#### BallNode(const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
- Initialize with a node created by the node name given as an argument

#### BallNode(const std::string & node_name, const std::string & node_namespace, const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
- Initialize with a node created by the node name and namespace given as arguments

#### explicit BallNode(rclcpp::Node::SharedPtr node)
- Initialize with a node

###### &copy; 2023 KIKS