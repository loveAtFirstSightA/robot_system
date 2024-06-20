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

#include <iostream>
#include <iomanip>
#include <ctime>
#include <sstream>
#include <string>

// 获取并格式化当前系统时间的函数
inline std::string getCurrentTime() {
    // 获取当前系统时间
    std::time_t now = std::time(nullptr);
    // 将时间转换为本地时间
    std::tm* local_time = std::localtime(&now);

    // 使用字符串流来格式化时间
    std::ostringstream oss;
    oss << std::put_time(local_time, "[%Y-%m-%d %H:%M:%S] ");
    return oss.str();
}

// 打印日志消息，附带当前系统时间
inline void logMessage(const std::string& message) {
    std::cout << "[" << getCurrentTime() << "] " << message << std::endl;
}

#endif // ITERATIVE_CLOSEST_POINT__LOGGER_HPP_
