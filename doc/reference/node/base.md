# [kiks_ssl_vision_bridge_ros](../../../README.md)/[reference](../index.md)/node/base

## Description
Convert the udp packets from ssl-vision with google protobuf and expose them as vision_detection and map topic.

default name : ssl_vision_bridge_base

## Parameters

#### udp.port
- Type : int64
- Default : 10006
- Description : Port for receiving udp packets from ssl-vision

#### udp.address
- Type : string
- Default : "224.5.23.2"
- Description : Address for receiving udp packets from ssl-vision

#### map.enable
- Type : bool
- Default : true
- Description : Decide whether to publish the map topic

#### map.resolution
- Type : double
- Default : 0.01
- Description : Decide whether to publish the map topic

#### map.wall_width
- Type : double
- Default : 0.1
- Description : Field and goal wall thickness [m]

#### map.frame_id
- Type : string
- Default : "map"
- Description : frame_id of map topic

## Related

### class
- [base_node](../class/base_node.md)

### executor
- [base](../executor/base.md)
- [robot](../executor/robot.md)
- [multi_robots](../executor/multi_robots.md)
- [ball](../executor/ball.md)
- [multi_robots_and_ball](../executor/multi_robots_and_ball.md)

###### &copy; 2023 KIKS