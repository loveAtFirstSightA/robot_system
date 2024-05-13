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

#include <cmath>
#include <iostream>
#include "robot_teleop/teleop_keyboard.hpp"

namespace robot_teleop
{

TeleopKeyboard::TeleopKeyboard()
: Node("robot_teleop_keyboard")
{
     cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
          "/bcr_bot/cmd_vel",
          10);
     timer_ = this->create_wall_timer(
          std::chrono::microseconds(100),
          std::bind(&TeleopKeyboard::timerCallback, this));
     key_scan_timer_ = this->create_wall_timer(
          std::chrono::microseconds(100),
          std::bind(&TeleopKeyboard::keyScanTimerCallabck, this));
}

TeleopKeyboard::~TeleopKeyboard()
{
}

void
TeleopKeyboard::timerCallback()
{
     auto msg = geometry_msgs::msg::Twist();
     msg.linear.x = v_;
     msg.angular.z = w_;
     cmd_vel_pub_->publish(msg);
     // RCLCPP_INFO(this->get_logger(), "vel v: %lf w: %lf", msg.linear.x, msg.angular.z);
}

void
TeleopKeyboard::keyScanTimerCallabck()
{
     char key;
     if (readKey(key)) {
          // 处理按键输入
          std::cout << "Pressed key: " << key << std::endl;
          switch (key) {
               case 'w': {
                    v_ = 0.25f;
               }break;
               case 'x': {
                    v_ = -0.25f;
               }break;
               case 'a': {
                    w_ = 45.0f / 180.0f * M_PI;
               }break;
               case 'd': {
                    w_ = -45.0f / 180.0f * M_PI;
               }break;
               case 's': {
                    v_ = 0.0f;
                    w_ = 0.0f;
               }break;
               default:
                    break;
          }
     }
}

bool
TeleopKeyboard::readKey(char& key) 
{
     struct termios oldt, newt;
     tcgetattr(STDIN_FILENO, &oldt);
     newt = oldt;
     newt.c_lflag &= ~(ICANON | ECHO);
     tcsetattr(STDIN_FILENO, TCSANOW, &newt);

     int bytesWaiting;
     ioctl(STDIN_FILENO, FIONREAD, &bytesWaiting);
     if (bytesWaiting > 0) {
          std::cin >> key;
          tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
          return true;
     }

     tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
     return false;
}


}  // namespace robot_teleop
