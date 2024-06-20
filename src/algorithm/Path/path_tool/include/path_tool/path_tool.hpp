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

#ifndef PATH_TOOL__PATH_TOOL_HPP_
#define PATH_TOOL__PATH_TOOL_HPP_

#include "rclcpp/rclcpp.hpp"

namespace path_tool
{
class PathTool : public rclcpp::Node
{
public:
    PathTool();
    ~PathTool();

private:

};
}  // namespace path_tool
#endif  // PATH_TOOL__PATH_TOOL_HPP_