# [kiks_ssl_vision_bridge_ros](../../../README.md)/[reference](../index.md)/node/ball_publisher

## Description
Publish the ball by the node that manages this node.

default name : ssl_bridge_vision_ball_publisher

## Parameters

#### tf.enable
- Type : bool
- Default : false
- Description : Decide whether to enable tf_broafcaster

note : This parameter is false by default to prevent communication between multiple robots

#### ball.frame_id
- Type : string
- Default : "vision/ball"
- Description : topic name and child_frame_id of tf

#### map.frame_id
- Type : string
- Default : "map"
- Description : header frame_id of topic and tf

## Related

### class
- [ball_publisher_node](../class/ball_publisher_node.md)

### node
- [receiver](receiver.md)

### executor
- [ssl_vision_bridge](../executor/ssl_vision_bridge.md)

###### &copy; 2023 KIKS