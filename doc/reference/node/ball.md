# [kiks_ssl_vision_bridge_ros](../../../README.md)/[reference](../index.md)/node/robot

## Description
Receive vision_detection topic, extract ball pose and publish.

default name : ssl_bridge_vision_ball

## Parameters

#### tf.enable
- Type : bool
- Default : false
- Description : Decide whether to enable tf_broafcaster

note : This parameter is false by default to prevent communication between multiple robots

#### frame_id
- Type : string
- Default : "ball"
- Description : child_frame_id of tf

## Related

### class
- [ball_node](../class/ball_node.md)

### executor
- [ball](../executor/ball)

###### &copy; 2023 KIKS