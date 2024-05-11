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

// 确定输出参数的数据结构和种类
// 机器人的当前目标位置 描述路径曲线的特征点 直线的起点和终点 曲线的控制点
// 使用 直线-圆弧-直线 或者 使用 贝塞尔曲线 或者 使用 回旋螺线

// 曲线的拼接原则： 线条拼接连接处曲率突变变小


#ifndef PATH_TRACKING__PURE_PURSUIT__PURE_PURSUIT_HPP_
#define PATH_TRACKING__PURE_PURSUIT__PURE_PURSUIT_HPP_

#include <vector>
#include <cmath>
#include "path_tracking/math_library/math_library.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"

namespace path_tracking
{
class PurePursuit
{
public:
    PurePursuit();
    ~PurePursuit();

    void handlePurepursuit(const geometry_msgs::msg::Vector3Stamped pose, double lookaheadDist, std::vector<geometry_msgs::msg::Vector3> points);
private:

};
}  // namespace path_tracking
#endif  // PATH_TRACKING__PURE_PURSUIT__PURE_PURSUIT_HPP_
