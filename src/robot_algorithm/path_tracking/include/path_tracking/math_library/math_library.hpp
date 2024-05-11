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

#ifndef PATH_TRACKING__MATH_LIBRARY__MATH_LIBRARY_HPP_
#define PATH_TRACKING__MATH_LIBRARY__MATH_LIBRARY_HPP_

#include <vector>
#include <cmath>

namespace path_tracking
{

class MathLibrary
{
public:
    MathLibrary();
    ~MathLibrary();

    // 计算向量的长度
    // 计算向量的方向


private:
    // 私有变量
    double x{0.0f};
    double y{0.0f};




};
}  // namespace path_tracking
#endif  // PATH_TRACKING__MATH_LIBRARY__MATH_LIBRARY_HPP_