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

#ifndef ROBOT_TELEOP__TELEOP_KEYBOARD_HPP_
#define ROBOT_TELEOP__TELEOP_KEYBOARD_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h> // 添加这一行

namespace robot_teleop
{
class TeleopKeyboard : public rclcpp::Node
{
public:
    TeleopKeyboard();
    ~TeleopKeyboard();

private:
    void timerCallback();
    void keyScanTimerCallabck();
    bool readKey(char& key);

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    double v_{0.0f};
    double w_{0.0f};
    rclcpp::TimerBase::SharedPtr key_scan_timer_;



};
}  // namespace robot_teleop
#endif  // ROBOT_TELEOP__TELEOP_KEYBOARD_HPP_
