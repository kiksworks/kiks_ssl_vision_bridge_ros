# [kiks_ssl_vision_bridge_ros](../../../README.md)/[reference](../index.md)/class/robot_publisher_node

#include "kiks_ssl_vision_bridge/robot_publisher_node.hpp"

kiks::ssl_vision_bridge::RobotPublisherNode

#### inheritance
- [ros_node_base](ros_node_base.md)

## Description
- This node provides a function to publish a robot to the receiver node. 
- Node alone does not work.

## public non-member function

#### static std::string default_name()
- Return default node name (="ssl_vision_bridge_robot_publisher")

## public member function

#### explicit RobotPublisherNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
- Initialize with a node created by the default node name (="ssl_vision_bridge_robot_publisher")

#### RobotPublisherNode(const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
- Initialize with a node created by the node name given as an argument

#### RobotPublisherNode(const std::string & node_name, const std::string & node_namespace, const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
- Initialize with a node created by the node name and namespace given as arguments

#### explicit RobotPublisherNode(rclcpp::Node::SharedPtr node)
- Initialize with a node

#### void publish_robot(const TimeMsg & stamp, const SSL_DetectionRobot & robot)
- Publish robot as geometry_msgs::msg::Pose.
- If tf is enabled, tf will also broadcast.

## Related

### class
- [receiver_node](receiver_node.md)

### node
- [robot_publisher](../node/robot_publisher.md)

###### &copy; 2023 KIKS