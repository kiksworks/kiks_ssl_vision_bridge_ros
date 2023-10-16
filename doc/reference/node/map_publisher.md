# [kiks_ssl_vision_bridge_ros](../../../README.md)/[reference](../index.md)/node/map_publisher

## Description
Publish the map by the node that manages this node.

default name : ssl_bridge_vision_map_publisher

## Parameters

#### map.resolution
- Type : double
- Default : 0.01
- Description : Distance per cell [m]

#### map.wall_width
- Type : double
- Default : 0.02
- Description : Field and goal thickness

#### map.frame_id
- Type : string
- Default : "map"
- Description : header frame_id of topic and tf

## Related

### class
- [map_publisher_node](../class/map_publisher_node.md)

### node
- [receiver](receiver.md)

### executor
- [ssl_vision_bridge](../executor/ssl_vision_bridge.md)

###### &copy; 2023 KIKS