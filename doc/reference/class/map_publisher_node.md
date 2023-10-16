# [kiks_ssl_vision_bridge_ros](../../../README.md)/[reference](../index.md)/class/map_publisher_node

#include "kiks_ssl_vision_bridge/ros_node_base.hpp"

kiks::ssl_vision_bridge::MapPublisherNode

#### inheritance
- [ros_node_base](ros_node_base.md)

## Description
- This node provides a function to publish a robot to the receiver node. 
- Node alone does not work.

## public non-member function

#### static std::string default_name()
- Return default node name (="ssl_vision_bridge_map_publisher")

## public member function

#### explicit MapPublisherNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
- Initialize with a node created by the default node name (="ssl_vision_bridge_map_publisher")

#### MapPublisherNode(const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
- Initialize with a node created by the node name given as an argument

#### MapPublisherNode(const std::string & node_name, const std::string & node_namespace, const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
- Initialize with a node created by the node name and namespace given as arguments

#### explicit MapPublisherNode(rclcpp::Node::SharedPtr node)
- Initialize with a node

#### void publish_map(const TimeMsg & stamp, const SSL_GeometryFieldSize & field)
- Publish map as nav_msgs::msg::OccupancyGrid.

## Related

### class
- [receiver_node](receiver_node.md)

### node
- [map_publisher](../node/map_publisher.md)

###### &copy; 2023 KIKS