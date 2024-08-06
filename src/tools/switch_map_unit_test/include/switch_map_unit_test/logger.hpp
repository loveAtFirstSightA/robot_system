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
#include <iomanip>
#include <sstream>
#include <string>
#include <ctime>

namespace switch_map_unit_test
{

inline std::string getCurrentTime() 
{
    auto now = std::chrono::system_clock::now();
    std::time_t now_time_t = std::chrono::system_clock::to_time_t(now);
    std::tm* local_time = std::localtime(&now_time_t);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
    std::ostringstream oss;
    oss << std::put_time(local_time, "[%Y-%m-%d %H:%M:%S");
    oss << '.' << std::setw(3) << std::setfill('0') << ms.count() << "] ";

    return oss.str();
}

}  //  namespace switch_map_unit_test

#endif // SWITCH_MAP_UNIT_TEST__LOGGER_HPP_
