# [kiks_ssl_vision_bridge_ros](../../../README.md)/[reference](../index.md)/class/robot_node

#include "kiks_ssl_vision_bridge/robot_node.hpp"

kiks::ssl_vision_bridge::RobotNode

#### inheritance
- [ros_node_base](ros_node_base.md)

## public non-member function

#### static std::string default_name()
- Return default node name (="ssl_bridge_vision_robot")

## public member function

#### explicit RobotNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
- Initialize with a node created by the default node name (="ssl_bridge_vision_robot")

#### RobotNode(const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
- Initialize with a node created by the node name given as an argument

#### RobotNode(const std::string & node_name, const std::string & node_namespace, const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
- Initialize with a node created by the node name and namespace given as arguments

#### explicit RobotNode(rclcpp::Node::SharedPtr node)
- Initialize with a node

## Related

### class
- [multi_robots_node](multi_robots_node.md)

###### &copy; 2023 KIKS