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

#include "spdlog_demo/spdlog_demo.hpp"

namespace spdlog_demo
{
SpdlogDemo::SpdlogDemo() : Node("spdlog_demog")
{
     RCLCPP_INFO(this->get_logger(), "ros2 default log ouput");
     // url: /home/lio/robot_system/temp
     
     // 设置日志文件路径
     auto logger = spdlog::basic_logger_mt("file_logger", "/home/lio/robot_system/temp/spdlog_demo.log");

     // 设置日志格式
     logger->set_pattern("[%H:%M:%S %z] [%n] [%^---%L---%$] [thread %t] %v");

     // 使用 logger 输出日志信息
     logger->info("Welcome to spdlog!");
     logger->error("Some error message with arg: {}", 1);
     logger->warn("Easy padding in numbers like {:08d}", 12);
     logger->critical("Support for int: {0:d};  hex: {0:x};  oct: {0:o}; bin: {0:b}", 42);
     logger->info("Support for floats {:03.2f}", 1.23456);
     logger->info("Positional args are {1} {0}..", "too", "supported");
     logger->info("{:<30}", "left aligned");

     spdlog::set_level(spdlog::level::debug); // 设置全局日志等级为 debug
     logger->debug("This message should be displayed..");    

     // 编译时日志级别
     SPDLOG_TRACE("Some trace message with param {}", 42);
     SPDLOG_DEBUG("Some debug message");


}

SpdlogDemo::~SpdlogDemo() {}







}  // namespace spdlog_demo
