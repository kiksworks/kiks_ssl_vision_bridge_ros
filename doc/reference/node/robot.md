# [kiks_ssl_vision_bridge_ros](../../../README.md)/[reference](../index.md)/node/robot

## Description
Receive vision_detection topic, extract robot pose and publish.

default name : ssl_bridge_vision_robot

## Parameters

#### team_color
- Type : string
- Default : "yellow"
- Description : Decide which color team's data to use

#### robot_id
- Type : int64
- Default : 0
- Description : Decide which robot_id data to use

#### tf.enable
- Type : bool
- Default : false
- Description : Decide whether to enable tf_broafcaster

note : This parameter is false by default to prevent communication between multiple robots

#### frame_id
- Type : string
- Default : "vision_footprint"
- Description : child_frame_id of tf

## Related

### class
- [robot_node](../class/robot_node.md)

### executor
- [robot](../executor/robot)
- [multi_robots](../executor/multi_robots)
- [multi_robots_and_ball](../executor/multi_robots_and_ball)

###### &copy; 2023 KIKS