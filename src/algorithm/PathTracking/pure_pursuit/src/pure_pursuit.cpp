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
     this->declare_parameter<double>("k", 0.3f);
     this->declare_parameter<double>("max_v", 1.0f);
     this->declare_parameter<double>("max_w", 1.57f);
     lookaheaddist_ = this->get_parameter("lookaheaddist").get_value<double>();
     k_ = this->get_parameter("k").get_value<double>();
     max_v_ = this->get_parameter("max_v").get_value<double>();
     max_w_ = this->get_parameter("max_w").get_value<double>();

     std::cout << getCurrentTime() << "Parameter:" << std::endl;
     std::cout << getCurrentTime() << "    lookaheaddist: " << lookaheaddist_ << std::endl;
     std::cout << getCurrentTime() << "    k: " << k_ << std::endl;
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
     this->v_ = 0.5f;
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

// main function of purepursuit algorithm 
void PurePursuit::currentPoseCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
{
     Pose current;
     current.x = msg->vector.x;
     current.y = msg->vector.y;
     current.yaw = msg->vector.z;

     // 切换路径跟踪逻辑,若是当前点与路径终点的距离小于前视距离，则使用下一条路径
     // 目标路径是四条直线路径（闭环矩形框）
     static unsigned int path_number = 0;
     double error_x = std::pow(line_[path_number].end.x - current.x, 2);
     double error_y = std::pow(line_[path_number].end.y - current.y, 2);
     double error_dist = std::sqrt(error_x + error_y);

     if (error_dist < lookaheaddist_) {
          path_number ++;
          if (path_number >= line_.size()) {
               path_number = 0;
          }
          std::cout << getCurrentTime() << "Tracking path number: " << path_number << std::endl;
     }

     // Pure Pursuit algorithm
     // Step 1 根据前视距离计算路径上的目标点
     Point target;
     Point closest;
     // Step 1.1 计算当前点到直线的最近点(投影比例系数)
     // 向量B Ps->Pe
     double vx_ps_pe = line_[path_number].end.x - line_[path_number].start.x;
     double vy_ps_pe = line_[path_number].end.y - line_[path_number].start.y;
     // 向量A Ps->Current
     double vx_ps_curr = current.x - line_[path_number].start.x;
     double vy_ps_curr = current.y - line_[path_number].start.y;
     // |B| * |B|
     double length_square = std::pow(vx_ps_pe, 2) + std::pow(vy_ps_pe, 2);
     // 计算投影比例系数 t
     double t = vx_ps_pe * vx_ps_curr + vy_ps_pe * vy_ps_curr / length_square;
     // 限制t范围[0, 1]
     closest.x = line_[path_number].start.x + t * vx_ps_pe;
     closest.y = line_[path_number].start.y + t * vy_ps_pe;
     // Step 1.2 
     // 路径向量长度
     double path_length = std::sqrt(vx_ps_pe * vx_ps_pe + vy_ps_pe * vy_ps_pe);
     // 单位向量
     double path_ux = vx_ps_pe / path_length; 
     double path_uy = vy_ps_pe / path_length;
     
     // 前视距离参数
     double lookaheaddistance = k_ * v_ + lookaheaddist_;

     // 目标点 = 当前点 + 前视距离 * 单位向量
     target.x = closest.x + lookaheaddistance * path_ux;
     target.y = closest.y + lookaheaddistance * path_uy;
     std::cout << getCurrentTime() << "clost:[" << closest.x << ", " << closest.y << ", target:[" << target.x << ", " << target.y << "]" << std::endl;
     // Step 2 根据当前点 目标点 当前角度 确定角度alpha
     double ld_vx = target.x - current.x;
     double ld_vy = target.y - current.y;
     double ld_theta = std::atan2(ld_vy, ld_vx);
     double alpha  = ld_theta - current.yaw;
     alpha = normalizeAngle(alpha);
     std::cout << getCurrentTime() << "alpha  = ld_theta - current.yaw " << alpha << " = " << ld_theta << " - " << current.yaw << std::endl;

     // Step 3 差速类型的模型计算旋转半径R
     double r = lookaheaddistance / (2.0f * std::sin(alpha));
    
     // Step 4 v = w * r
     w_ = v_ / r;

     std::cout << getCurrentTime() << "r: " << r << ", w: " << w_ << std::endl;
     sendVelocity(v_, w_);
}

void PurePursuit::sendVelocity(const double v, const double w)
{
     auto msg = geometry_msgs::msg::Twist();
     msg.linear.x = v;
     msg.angular.z = w;
     vel_->publish(msg);
}




}  // namespace pure_pursuit
