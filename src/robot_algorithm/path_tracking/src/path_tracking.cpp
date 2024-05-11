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

#include "path_tracking/path_tracking.hpp"

namespace path_tracking
{
PathTracking::PathTracking()
: Node("path_tracking")
{
     RCLCPP_INFO(this->get_logger(), "path_tracking is running...");
     initSubPub();
     initTimer();

     // debug
     line_s_.x = 1.76f;
     line_s_.y = 0.0f;
     line_e_.x = 1.76f;
     line_e_.y = -9.0f;
}

PathTracking::~PathTracking()
{

}

void
PathTracking::initSubPub()
{
     current_pose_sub_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
          "robot_pose",
          rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
          std::bind(&PathTracking::currentPoseSubCallback, this, std::placeholders::_1));
     vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
          "/bcr_bot/cmd_vel",
          10);
     
     RCLCPP_INFO(this->get_logger(), "All Subscribers Publishers initial successfully");
}

void
PathTracking::initTimer()
{
     timer_ = this->create_wall_timer(
          std::chrono::microseconds(20),
          std::bind(&PathTracking::timerCallback, this));
}

void
PathTracking::timerCallback()
{
     double w;
     // debug
     w = -0.1f;
     // 控制角速度解算和发布

     // 调用纯跟踪算法

     // 发布速度指令 T = 20ms
     sendVelocity(0.0f, w);
}

void
PathTracking::currentPoseSubCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
{
     // 订阅机器人当前的位姿
     current_pose_ = *msg;
     static int count;
     if ( (count ++) == 25000 ) {
          count = 0;
          RCLCPP_INFO(this->get_logger(), "current pose [%lf, %lf, %lf]", 
               current_pose_.vector.x, current_pose_.vector.y, current_pose_.vector.z);
     }
}

void
PathTracking::sendVelocity(double v, double w)
{
     auto msg = geometry_msgs::msg::Twist();
     msg.linear.x = v;
     msg.angular.z = w;
     vel_pub_->publish(msg);
}








}  // namespace path_tracking

