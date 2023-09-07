# [kiks_ssl_vision_bridge_ros](../../../README.md)/[reference](../index.md)/class/multi_node

#include "kiks_ssl_vision_bridge/multi_node.hpp"

template&lt;class BaseNode&gt; kiks::ssl_vision_bridge::MultiNode

#### inheritance
- [ros_node_base](ros_node_base.md)

## public member function

#### explicit MultiNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
- Initialize with a node created by the default node name of BaseNode
- Generate subnodes with multiple strings given in "namespaces"

#### MultiNode(const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
- Initialize with a node created by the node name given as an argument
- Generate subnodes with multiple strings given in "namespaces"

#### MultiNode(const std::string & node_name, const std::string & node_namespace, const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
- Initialize with a node created by the node name and namespace given as arguments
- Generate subnodes with multiple strings given in "namespaces"

#### explicit MultiNode(rclcpp::Node::SharedPtr node)
- Initialize with a node
- Generate subnodes with multiple strings given in "namespaces"

## Related

### class
- [multi_robots_node](multi_robots_node.md)

###### &copy; 2023 KIKS