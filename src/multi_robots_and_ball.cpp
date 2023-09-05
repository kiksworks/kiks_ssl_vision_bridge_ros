// Copyright 2023 KIKS
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.


#include "rclcpp/context.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"

#include "kiks_ssl_vision_bridge/ball_node.hpp"
#include "kiks_ssl_vision_bridge/base_node.hpp"
#include "kiks_ssl_vision_bridge/multi_robots_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  kiks::ssl_vision_bridge::BaseNode base_node;
  kiks::ssl_vision_bridge::MultiRobotsNode robots_node;
  kiks::ssl_vision_bridge::BallNode ball_node;
  exec.add_node(base_node);
  exec.add_node(robots_node);
  exec.add_node(ball_node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
