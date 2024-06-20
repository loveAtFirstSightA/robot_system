/*
 Copyright 2024 ROS2 LLC

 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at

      https://www.apache.org/licenses/LICENSE-2.0

 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.
 */

#include "path_tool/path_tool.hpp"

namespace path_tool
{
PathTool::PathTool() : Node ("path_tool")
{
     timer_ = this->create_wall_timer(
          std::chrono::seconds(1),
          std::bind(&PathTool::timerCallback, this));
     paths_pub_ = this->create_publisher<algorithm_msgs::msg::Path>(
          "algorithm_path",
          rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
}

PathTool::~PathTool() {}

void PathTool::timerCallback()
{
     // 初始化路径
     // 显示路径到rviz2
}

bool PathTool::initPaths()
{
     // 初始化路径信息




     return true;
}


}  // namespace path_tool
