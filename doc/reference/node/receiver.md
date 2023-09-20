# [kiks_ssl_vision_bridge_ros](../../../README.md)/[reference](../index.md)/node/receiver

## Description
Convert udp data from ssl-vision and publish by publisher in sub_node.

default name : ssl_vision_bridge

## Sub nodes
- [robot_publisher](robot_publisher.md)
- [ball_publisher](ball_publisher.md)
- [map_publisher](map_publisher.md)

## Parameters

#### udp.port
- Type : int64
- Default : 10006
- Description : Port for receiving udp packets from ssl-vision

#### udp.address
- Type : string
- Default : "224.5.23.2"
- Description : Address for receiving udp packets from ssl-vision

#### udp.interface
- Type : string
- Default : ""
- Description : Interface for receiving udp packets from ssl-vision

#### yellow_robots
- Type : string array
- Default : {"yellow0", "yellow1", ... , "yellow14", "yellow15"}
- Description : Yellow_team robots("" will result in an invalid robot)

#### blue_robots
- Type : string array
- Default : {"bulue0", "bulue1", ... , "bulue14", "bulue15"}
- Description : Blue_team robots("" will result in an invalid robot)

#### ball.enable
- Type : bool
- Default : true
- Description : Deciding to enable ball publisher node

#### map.enable
- Type : bool
- Default : true
- Description :Deciding to enable map publisher node

## Related

### class
- [receiver_node](../class/base_node.md)

### node
- [robot_publisher](robot_publisher.md)
- [ball_publisher](ball_publisher.md)
- [map_publisher](map_publisher.md)

### executor
- [ssl_vision_bridge](../executor/ssl_vision_bridge.md)

###### &copy; 2023 KIKS