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

// How to use this executable program
// nohup ./data_recorder > data_recorder_2024_0620_0000.txt 2>&1 &

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "data_recorder/data_recorder.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<data_recorder::DataRecorder>());
    rclcpp::shutdown();
    return 0;
}
