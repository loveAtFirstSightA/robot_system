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


#ifndef ROBOT_PATH_CREATER__PATH_CREATER_HPP_
#define ROBOT_PATH_CREATER__PATH_CREATER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "robot_path_creater/curve_math.hpp"

namespace path_creater
{
class PathCreater : public rclcpp::Node
{
public:
    PathCreater();
    ~PathCreater();

private:

};
}  // namespace path_creater
#endif  // ROBOT_PATH_CREATER__PATH_CREATER_HPP_
