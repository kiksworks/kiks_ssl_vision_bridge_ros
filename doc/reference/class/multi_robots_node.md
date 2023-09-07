# [kiks_ssl_vision_bridge_ros](../../../README.md)/[reference](../index.md)/class/multi_robots_node

#include "kiks_ssl_vision_bridge/multi_robots_node.hpp"

kiks::ssl_vision_bridge::MultiRobotsNode

#### inheritance
- [ros_node_base](ros_node_base.md)

## public non-member function

#### static std::string default_name()
- Return default node name (="ssl_bridge_vision_ssl_bridge_vision_multi_robotsrobot")

## public member function

#### explicit MultiRobotsNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
- Initialize with a node created by the default node name (="ssl_bridge_vision_multi_robots")

#### MultiRobotsNode(const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
- Initialize with a node created by the node name given as an argument

#### MultiRobotsNode(const std::string & node_name, const std::string & node_namespace, const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
- Initialize with a node created by the node name and namespace given as arguments

#### explicit MultiRobotsNode(rclcpp::Node::SharedPtr node)
- Initialize with a node

## Related

### class
- [multi_node](multi_node.md)
- [robot_node](robot_node.md)

###### &copy; 2023 KIKS