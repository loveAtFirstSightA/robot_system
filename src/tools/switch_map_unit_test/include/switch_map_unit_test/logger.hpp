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

#ifndef SWITCH_MAP_UNIT_TEST__LOGGER_HPP_
#define SWITCH_MAP_UNIT_TEST__LOGGER_HPP_

#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <string>
#include <iostream>

namespace switch_map_unit_test
{

enum LogLevel {
    INFO,
    DEBUG,
    WARNING,
    ERROR
};

inline std::string LOG(LogLevel level) 
{
    // 获取当前系统时间点
    auto now = std::chrono::system_clock::now();
    
    // 将系统时间点转换为time_t格式
    std::time_t now_time_t = std::chrono::system_clock::to_time_t(now);
    
    // 将time_t格式时间转换为本地时间（tm结构体）
    std::tm local_time;
    localtime_r(&now_time_t, &local_time);  // 使用线程安全的localtime_r
    
    // 计算自epoch以来的毫秒数，并取当前秒内的毫秒数
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
    
    // 创建一个大小足够的字符串
    char buffer[32];  // 根据时间格式长度合理设置缓冲区大小
    
    // 将本地时间格式化为字符串
    std::strftime(buffer, sizeof(buffer), "[%Y-%m-%d %H:%M:%S", &local_time);
    
    // 创建一个字符串输出流，并格式化毫秒数
    std::ostringstream oss;
    oss << buffer << '.' << std::setw(3) << std::setfill('0') << ms.count() << "] ";

    // 添加日志等级
    switch(level) {
        case INFO:
            oss << "[INFO] ";
            break;
        case DEBUG:
            oss << "[DEBUG] ";
            break;
        case WARNING:
            oss << "[WARNING] ";
            break;
        case ERROR:
            oss << "[ERROR] ";
            break;
    }

    // 返回格式化后的时间字符串
    return oss.str();
}

}  //  namespace switch_map_unit_test

#endif // SWITCH_MAP_UNIT_TEST__LOGGER_HPP_
