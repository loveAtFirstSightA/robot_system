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

#ifndef ITERATIVE_CLOSEST_POINT__LOGGER_HPP_
#define ITERATIVE_CLOSEST_POINT__LOGGER_HPP_

#include <chrono>
#include <iomanip>
#include <sstream>
#include <string>
#include <ctime>

// inline std::string getCurrentTime() 
// {
//     std::time_t now = std::time(nullptr);
//     std::tm* local_time = std::localtime(&now);
//     std::ostringstream oss;
//     oss << std::put_time(local_time, "[%Y-%m-%d %H:%M:%S] ");
//     return oss.str();
// }

inline std::string getCurrentTime() 
{
    // 获取当前时间点
    auto now = std::chrono::system_clock::now();
    
    // 转换为 time_t 类型以获取本地时间
    std::time_t now_time_t = std::chrono::system_clock::to_time_t(now);
    
    // 转换为 tm 结构体以便格式化
    std::tm* local_time = std::localtime(&now_time_t);
    
    // 获取当前时间点的小数部分（即毫秒）
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
    
    // 使用输出字符串流来格式化时间
    std::ostringstream oss;
    oss << std::put_time(local_time, "[%Y-%m-%d %H:%M:%S");
    
    // 添加毫秒部分
    oss << '.' << std::setw(3) << std::setfill('0') << ms.count() << "] ";
    
    // 返回格式化的字符串
    return oss.str();
}

#endif // ITERATIVE_CLOSEST_POINT__LOGGER_HPP_
