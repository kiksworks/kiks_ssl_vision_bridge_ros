# [kiks_ssl_vision_bridge_ros](../../../README.md)/[reference](../index.md)/class/ball_publisher_node

#include "kiks_ssl_vision_bridge/ball_publisher_node.hpp"

kiks::ssl_vision_bridge::BallPublisherNode

#### inheritance
- [ros_node_base](ros_node_base.md)

## Description
- This node provides a function to publish a ball to the receiver node. 
- Node alone does not work.

## public type
#### TimeMsg(= builtin_interfaces::msg::Time)

## public non-member function

#### static std::string default_name()
- Return default node name (="ssl_vision_bridge_ball_publisher")

## public member function

#### explicit BallPublisherNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
- Initialize with a node created by the default node name (="ssl_vision_bridge_ball_publisher")

#### BallPublisherNode(const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
- Initialize with a node created by the node name given as an argument

#### BallPublisherNode(const std::string & node_name, const std::string & node_namespace, const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
- Initialize with a node created by the node name and namespace given as arguments

#### explicit BallPublisherNode(rclcpp::Node::SharedPtr node)
- Initialize with a node

#### void publish_ball(const TimeMsg & stamp, const SSL_DetectionBall & ball)
- Publish ball as geometry_msgs::msg::Point.
- If tf is enabled, tf will also broadcast.

## Related

### class
- [receiver_node](receiver_node.md)

### node
- [ball_publisher](../node/ball_publisher.md)

###### &copy; 2023 KIKS