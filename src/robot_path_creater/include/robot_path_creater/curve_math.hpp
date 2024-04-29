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


#ifndef ROBOT_PATH_CREATER__CURVE_MATH__CURVE_MATH_HPP_
#define ROBOT_PATH_CREATER__CURVE_MATH__CURVE_MATH_HPP_

#include <vector>
#include <cmath>
#include "geometry_msgs/msg/vector3.hpp"

namespace robot_path_creater
{

class CurveMath
{
public:
     CurveMath();
     ~CurveMath();

     std::vector<geometry_msgs::msg::Vector3> line(double start_x, double start_y, double end_x, double end_y, double resolution);

private:
     

};
}  // namespace robot_path_creater
#endif  // ROBOT_PATH_CREATER__CURVE_MATH__CURVE_MATH_HPP_
