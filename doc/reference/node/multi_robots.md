# [kiks_ssl_vision_bridge_ros](../../../README.md)/[reference](../index.md)/node/multi_robots

## Description
Receive vision_detection topic, extract multi robots pose and publish.

default name : ssl_bridge_vision_multi_robots

## Sub Node
- [robot](robot.md)

## Parameters

#### namespaces
- Type : string array
- Default : ["yellow0", "yellow1" ... "blue9", "blue10"]
- Description : Decide which color team's data to use

#### team_color or ${namespace}.team_color
- Type : string
- Default : "yellow"
- Description : Decide which color team's data to use

#### robot_id or ${namespace}.robot_id
- Type : int64
- Default : 0
- Description : Decide which robot_id data to use

#### tf.enable or ${namespace}.tf.enable
- Type : bool
- Default : false
- Description : Decide whether to enable tf_broafcaster

note : This parameter is false by default to prevent communication between multiple robots

#### frame_id or ${namespace}.frame_id
- Type : string
- Default : "vision_footprint"
- Description : child_frame_id of tf

## Related

### class
- [multi_robots_node](../class/multi_robots_node.md)

### executor
- [multi_robots](../executor/multi_robots)
- [multi_robots_and_ball](../executor/multi_robots_and_ball)

###### &copy; 2023 KIKS