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
#include <algorithm>
#include "robot_laser_scan/laser_scan.hpp"

namespace laser_scan
{
LaserScan::LaserScan()
: Node("robot_laser_scan_node")
{
     laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
          // "/bcr_bot/scan",
          "scan",
          1,
          std::bind(&LaserScan::laserScanSubCallback, this, std::placeholders::_1));
}

LaserScan::~LaserScan()
{

}

void
LaserScan::laserScanSubCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
     RCLCPP_INFO(this->get_logger(), "%s - %.9d:%.9d - angle [min %.4f ~ max %.4f inc %.4f] - range [min %.4f ~ max %.4f          0 val %.4f] - scan_time %.4f - time_inc %.4f - intens %.4f",
          msg->header.frame_id.c_str(), msg->header.stamp.sec, msg->header.stamp.nanosec,
          msg->angle_min, msg->angle_max, msg->angle_increment, msg->range_min, msg->range_max, 
          msg->ranges[0], msg->scan_time, msg->time_increment, msg->intensities[0]);
     RCLCPP_INFO(this->get_logger(), "%s - %.9d:%.9d - angle [min %.4f ~ max %.4f inc %.4f] - range [min %.4f ~ max %.4f max * 0.25 val %.4f] - scan_time %.4f - time_inc %.4f - intens %.4f",
          msg->header.frame_id.c_str(), msg->header.stamp.sec, msg->header.stamp.nanosec,
          msg->angle_min, msg->angle_max, msg->angle_increment, msg->range_min, msg->range_max, 
          msg->ranges[static_cast<unsigned int>(msg->ranges.size() * 0.25f)], msg->scan_time, msg->time_increment, msg->intensities[static_cast<unsigned int>(msg->ranges.size() * 0.25f)]);
     RCLCPP_INFO(this->get_logger(), "%s - %.9d:%.9d - angle [min %.4f ~ max %.4f inc %.4f] - range [min %.4f ~ max %.4f  max * 0.5 val %.4f] - scan_time %.4f - time_inc %.4f - intens %.4f",
          msg->header.frame_id.c_str(), msg->header.stamp.sec, msg->header.stamp.nanosec,
          msg->angle_min, msg->angle_max, msg->angle_increment, msg->range_min, msg->range_max, 
          msg->ranges[static_cast<unsigned int>(msg->ranges.size() * 0.5f)], msg->scan_time, msg->time_increment, msg->intensities[static_cast<unsigned int>(msg->ranges.size() * 0.5f)]);
     
     // 欧式坐标系
     for (size_t i = 0; i < msg->ranges.size(); i ++) {
          // 确定扫描角度
          float angle = msg->angle_min + msg->angle_increment * i;
          // 三角函数
          float x = msg->ranges.at(i) * std::cos(angle);
          float y = msg->ranges.at(i) * std::sin(angle);
          RCLCPP_DEBUG(this->get_logger(), "Num %ld - [x %.4f, y %.4f]", i, x, y);
     }
     // red
     RCLCPP_INFO(this->get_logger(), "\x1b[31mRed\x1b[0m");
     // green
     RCLCPP_INFO(this->get_logger(), "\x1b[32mGreen\x1b[0m");
     // Orange
     RCLCPP_INFO(this->get_logger(), "\x1b[33mOrange\x1b[0m");
     // blue
     RCLCPP_INFO(this->get_logger(), "\x1b[34mBlue\x1b[0m");
     // show type-int
     RCLCPP_INFO(this->get_logger(), "\x1b[31mShow type-int: %d\x1b[0m", 1);
     // show type-float
     RCLCPP_INFO(this->get_logger(), "\x1b[32mShow type-float: %f\x1b[0m", 1.11);
}



}  // namespace laser_scan
