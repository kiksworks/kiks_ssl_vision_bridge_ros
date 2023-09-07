# kiks_ssl_vision_bridge_ros

## Description
Bridge UDP packets from ssl-vision to ROS2 topic

## Supported environment
- ubuntu 22.04

note : This package may work on ubuntu20.04 or 18.04, but they are not supported.

## Required packages
- Git ([view installation](https://git-scm.com/book/en/v2/Getting-Started-Installing-Git))
- ROS2 humble ([view installation](https://docs.ros.org/en/humble/Installation.html))
- colcon ([view installation](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html))
- Qt6 or Qt5 (Network)
- Protocol Buffers ([view installation](https://github.com/protocolbuffers/protobuf#protobuf-compiler-installation))

## Installation
1. install required packages
2. create work space and source directory
```
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
```
note : ros2_ws can be replaced with any name you like.

note : If you have an existing workspace you can use it.

3. clone this repository
```
git clone --recursive https://github.com/kiksworks/kiks_ssl_vision_bridge_ros.git kiks_ssl_vision_bridge
```
note : If you forget --recursive when cloning, please run "git submodule update --init --recursive".

4. move to work space directory
```
cd ..
```
5. build
```
colcon build
```

## Run
1. source setup
```
source ~/ros2_ws/install/setup.bash
```
2. run
```
ros2 run kiks_ssl_vision_bridge multi_robots_and_ball
```

## Document
- [reference](doc/reference/index.md)
<!-- - [guide](doc/guide/index.md) -->

## To contribute
read [CONTRIBUTING.md](CONTRIBUTING.md)

## Contact
- email : kiks.nittc@gmail.com

## License
- GPLv3([view license file](LICENSE))

###### &copy; 2023 KIKS