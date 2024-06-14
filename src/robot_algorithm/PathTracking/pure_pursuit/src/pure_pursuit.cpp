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

// Path
// [1.760, 0.0]    [1.760, -9.45]
// [-3.75, 0.0]    [-3.75, -9.45]

#include <chrono>
#include "pure_pursuit/logger.hpp"
#include "pure_pursuit/pure_pursuit.hpp"

namespace pure_pursuit
{
PurePursuit::PurePursuit() : Node("PurePursuit")
{
     initPath();
     initParam();
     current_pose_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
          "estimate_pose",
          rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
          std::bind(&PurePursuit::currentPoseCallback, this, std::placeholders::_1));
     vel_ = this->create_publisher<geometry_msgs::msg::Twist>(
          "cmd_vel",
          10);
}

PurePursuit::~PurePursuit() {}

void PurePursuit::initParam()
{
     this->declare_parameter<double>("lookaheaddist", 0.15f);
     this->declare_parameter<double>("max_v", 1.0f);
     this->declare_parameter<double>("max_w", 1.57f);
     lookaheaddist_ = this->get_parameter("lookaheaddist").get_value<double>();
     max_v_ = this->get_parameter("max_v").get_value<double>();
     max_w_ = this->get_parameter("max_w").get_value<double>();

     std::cout << getCurrentTime() << "Parameter:" << std::endl;
     std::cout << getCurrentTime() << "    lookaheaddist: " << lookaheaddist_ << std::endl;
     std::cout << getCurrentTime() << "    max_v: " << max_v_ << std::endl;
     std::cout << getCurrentTime() << "    max_w: " << max_w_ << std::endl;
}

void PurePursuit::initPath()
{
     // Initialize path information
     unsigned int line_sum = 4;
     line_.clear();
     line_.resize(line_sum);
     line_[0].start.x = 1.760f;
     line_[0].start.y = 0.0f;
     line_[0].end.x = 1.760f;
     line_[0].end.y = -9.45f;
     line_[1].start.x = 1.760f;
     line_[1].start.y = -9.45f;
     line_[1].end.x = -3.75f;
     line_[1].end.y = -9.45f;
     line_[2].start.x = -3.75f;
     line_[2].start.y = -9.45f;
     line_[2].end.x = -3.75f;
     line_[2].end.y = 0.0f;
     line_[3].start.x = -3.75f;
     line_[3].start.y = 0.0f;
     line_[3].end.x = 1.760f;
     line_[3].end.y = 0.0f;
     std::cout << getCurrentTime() << "Created fixed paths" << std::endl;
     for (size_t i = 0; i < line_.size(); i++) {
          std::cout << getCurrentTime() << "    line " << i << " [" << line_[i].start.x << ", " << line_[i].start.y << "] --- [" 
               << line_[i].end.x << ", " << line_[i].end.y << "]" << std::endl;
     }
}

void PurePursuit::currentPoseCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
{
     double current_x = msg->vector.x;
     double current_y = msg->vector.y;
     double current_theta = msg->vector.z;

     double e_y;    //   横向偏差
     // std::cout << getCurrentTime() << "Current pose: [" << msg->vector.x << ", " << msg->vector.y << ", " << msg->vector.z << "]" << std::endl
     // 1、计算横向偏差 ey
     double vx = line_[0].end.x - line_[0].start.x;
     double vy = line_[0].end.y - line_[0].start.y;



     double w;
     w = 2.0f * v_ * e_y / std::pow(lookaheaddist_, 2);
}






}  // namespace pure_pursuit
