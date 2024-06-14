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
#include <cmath>
#include "pure_pursuit/logger.hpp"
#include "pure_pursuit/pure_pursuit.hpp"

namespace pure_pursuit
{
PurePursuit::PurePursuit() : Node("PurePursuit")
{
     // 初始化路径信息
     initPath();
     // 初始化参数信息
     initParam();
     // 初始化初值
     initFirstValue();
     // 当前车体中心的位姿
     current_pose_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
          "estimate_pose",
          rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
          std::bind(&PurePursuit::currentPoseCallback, this, std::placeholders::_1));
     // 显示直线路径
     marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
          "path_marker",
          10);
     // 目标速度
     vel_ = this->create_publisher<geometry_msgs::msg::Twist>(
          "cmd_vel",
          10);
     timer_ = this->create_wall_timer(
          std::chrono::seconds(1),
          std::bind(&PurePursuit::timerCallback, this));
}

PurePursuit::~PurePursuit() 
{
     sendVelocity(0.0f, 0.0f);
}

void PurePursuit::timerCallback()
{
     displayCurveOnRviz2();
}

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
     line_[0].start.x = 1.76f;
     line_[0].start.y = 0.0f;
     line_[0].end.x = 1.76f;
     line_[0].end.y = -9.45f;

     line_[1].start.x = 1.76f;
     line_[1].start.y = -9.45f;
     line_[1].end.x = -3.75f;
     line_[1].end.y = -9.45f;

     line_[2].start.x = -3.75f;
     line_[2].start.y = -9.45f;
     line_[2].end.x = -3.75f;
     line_[2].end.y = 0.0f;

     line_[3].start.x = -3.75f;
     line_[3].start.y = 0.0f;
     line_[3].end.x = 1.76f;
     line_[3].end.y = 0.0f;
     std::cout << getCurrentTime() << "Created fixed paths" << std::endl;
     for (size_t i = 0; i < line_.size(); i++) {
          std::cout << getCurrentTime() << "    line " << i << " [" << line_[i].start.x << ", " << line_[i].start.y << "] --- [" 
               << line_[i].end.x << ", " << line_[i].end.y << "]" << std::endl;
     }
}

void PurePursuit::displayCurveOnRviz2()
{
     for (size_t i = 0; i < line_.size(); i++) {
          visualization_msgs::msg::Marker line_marker;
          line_marker.header.frame_id = "map";
          line_marker.header.stamp = this->get_clock()->now();
          line_marker.ns = "lines";
          line_marker.id = i;
          line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
          line_marker.action = visualization_msgs::msg::Marker::ADD;
          line_marker.scale.x = 0.05; // Line width
          line_marker.color.r = 0.0;
          line_marker.color.g = 0.0;
          line_marker.color.b = 1.0;
          line_marker.color.a = 1.0;

          geometry_msgs::msg::Point p_start, p_end;
          p_start.x = line_[i].start.x;
          p_start.y = line_[i].start.y;
          p_start.z = 0.0; // Assume flat ground

          p_end.x = line_[i].end.x;
          p_end.y = line_[i].end.y;
          p_end.z = 0.0; // Assume flat ground

          line_marker.points.push_back(p_start);
          line_marker.points.push_back(p_end);

          marker_pub_->publish(line_marker);
     }
}

void PurePursuit::initFirstValue()
{
     this->v_ = 0.0f;
}

double PurePursuit::normalizeAngle(double angle)
{
     
     while (angle > M_PI) {
          angle -= 2.0f * M_PI;
     }
     while (angle < -M_PI) {
          angle += 2.0f * M_PI;
     }
     return angle; 
}

void PurePursuit::currentPoseCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
{
     double current_x = msg->vector.x;
     double current_y = msg->vector.y;
     double current_theta = msg->vector.z;

     // 切换路径跟踪逻辑,若是当前点与路径终点的距离小于前视距离，则使用下一条路径
     static unsigned int path_number = 0;
     double x_x = (line_[path_number].end.x - current_x) * (line_[path_number].end.x - current_x);
     double y_y = (line_[path_number].end.y - current_y) * (line_[path_number].end.y - current_y);
     double error_dist = std::sqrt(x_x + y_y);
     if (error_dist < lookaheaddist_) {
          path_number ++;
          if (path_number > line_.size()) {
               path_number = 0;
          }
          std::cout << getCurrentTime() << "Tracking path number: " << path_number << std::endl;
     }
     // Pure Pursuit algorithm
     // 1、根据前视距离计算路径上的目标点
     double target_x, target_y;
     // 确定路径的向量
     double vx = line_[path_number].end.x - line_[path_number].start.x;
     double vy = line_[path_number].end.y - line_[path_number].start.y;
     double length = std::sqrt(vx * vx + vy * vy);
     // double vtheta = std::atan2(vy, vx);     //   vtheta = std::atan2(4, 3) ≈ 0.93 radians
     double ux = vx / length;
     double uy = vy / length;
     double step = 0.01f;
     for (double t = 0; t <= length; t += step) {
          Point p;
          p.x = line_[path_number].start.x + t * ux;
          p.y = line_[path_number].start.y + t * uy;
          double dist = std::sqrt(std::pow(p.x - current_x, 2) + std::pow(p.y -  current_y, 2));
          double error_dist = fabs(lookaheaddist_ - dist);
          double threshold = 0.01f;
          if (error_dist < threshold) {
               target_x = p.x;
               target_y = p.y;
               break;
          }
     }
     // std::cout << getCurrentTime() << "find target point [" << target_x << ", " << target_y << "]" << std::endl;

     // 2、根据当前点 目标点 当前角度 确定角度alpha
     // 3、确定横向偏差e_y
     // 4、计算w = 2 * v * e_y / ld^2



     // debug
     // v_ = 0.0f;
     // sendVelocity(v_, w_);
}

void PurePursuit::sendVelocity(const double v, const double w)
{
     auto msg = geometry_msgs::msg::Twist();
     msg.linear.x = v;
     msg.angular.z = w;
     vel_->publish(msg);
}







}  // namespace pure_pursuit
