# [kiks_ssl_vision_bridge_ros](../../../README.md)/[reference](../index.md)/node/robot_publisher

## Description
Publish the robot by the node that manages this node.

default name : ssl_bridge_vision_robot_publisher

## Parameters

#### tf.enable
- Type : bool
- Default : false
- Description : Decide whether to enable tf_broafcaster

note : This parameter is false by default to prevent communication between multiple robots

#### robot.frame_id
- Type : string
- Default : "vision/base_footprint"
- Description : topic name and child_frame_id of tf

#### map.frame_id
- Type : string
- Default : "map"
- Description : header frame_id of topic and tf

## Related

### class
- [robot_publisher_node](../class/robot_node.md)

### node
- [receiver](receiver.md)

### executor
- [ssl_vision_bridge](../executor/ssl_vision_bridge.md)

###### &copy; 2023 KIKS