// Copyright 2023 KIKS.
//
// Licensed under the GNU GENERAL PUBLIC LICENSE, Version 3.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.gnu.org/licenses/gpl-3.0.html
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "kiks_ssl_vision_bridge/ball_node.hpp"
#include "kiks_ssl_vision_bridge/base_node.hpp"
#include "kiks_ssl_vision_bridge/multi_node.hpp"
#include "kiks_ssl_vision_bridge/robot_node.hpp"
#include "rclcpp/context.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  using namespace kiks::ssl_vision_bridge;
  BaseNode base_node;
  MultiNode<RobotNode> robots_node;
  BallNode ball_node;
  exec.add_node(base_node);
  exec.add_node(robots_node);
  exec.add_node(ball_node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
