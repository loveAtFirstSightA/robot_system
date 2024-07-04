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

#ifndef MODEL_PREDICT_CONTROL__LOGGER_HPP_
#define MODEL_PREDICT_CONTROL__LOGGER_HPP_

#include <iostream>
#include <iomanip>
#include <ctime>
#include <sstream>
#include <string>

inline std::string getCurrentTime() 
{
    std::time_t now = std::time(nullptr);
    std::tm* local_time = std::localtime(&now);
    std::ostringstream oss;
    oss << std::put_time(local_time, "[%Y-%m-%d %H:%M:%S] ");
    return oss.str();
}

inline void logMessage(const std::string& message) 
{
    std::cout << "[" << getCurrentTime() << "] " << message << std::endl;
}

#endif // MODEL_PREDICT_CONTROL__LOGGER_HPP_
