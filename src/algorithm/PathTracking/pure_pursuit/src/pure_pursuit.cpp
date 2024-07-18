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
     initParam();
     initFirstValue();

     path_sub_ = this->create_subscription<algorithm_msgs::msg::Path>(
          "algorithm_path",
          rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
          std::bind(&PurePursuit::pathSubCallback, this, std::placeholders::_1));
     vel_ = this->create_publisher<geometry_msgs::msg::Twist>(
          "cmd_vel",
          10);
     
     tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
     tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
     timer_ = this->create_wall_timer(
          std::chrono::milliseconds(20),
          std::bind(&PurePursuit::timerCallback, this));
}

PurePursuit::~PurePursuit() 
{
     sendVelocity(0.0f, 0.0f);
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

void PurePursuit::initFirstValue()
{
     this->v_ = 0.5f;
}

bool PurePursuit::getCurrentPose(double & x, double & y, double & yaw)
{
     std::string target_link = "map";
     std::string source_link = "base_footprint";
     geometry_msgs::msg::TransformStamped tf;
     try {
          tf = tf_buffer_->lookupTransform(target_link, source_link, tf2::TimePointZero);
          x = tf.transform.translation.x;
          y = tf.transform.translation.y;
          yaw = tf2::getYaw(tf.transform.rotation);
          return true;
     } catch(const std::exception& e) {
          std::cerr << e.what() << '\n';
     }
     return false;
}

void PurePursuit::timerCallback()
{
     if (!is_path_received_) {
          std::cout << getCurrentTime() << "path is empty, return" << std::endl;
          sendVelocity(0.0f, 0.0f);
          return;
     }

     Pose current;
     if (!getCurrentPose(current.x, current.y, current.yaw)) {
          return;
     }
     // 切换路径
     static unsigned int path_number = 0;
     Pose path_end;
     if (path_.segments[path_number].type == path_.segments[path_number].LINE) {
          path_end.x = path_.segments[path_number].line.p1.x;
          path_end.y = path_.segments[path_number].line.p1.y;
     } else if (path_.segments[path_number].type == path_.segments[path_number].BEZIER3) {
          path_end.x = path_.segments[path_number].bezier3.p3.x;
          path_end.y = path_.segments[path_number].bezier3.p3.y;
     } else if (path_.segments[path_number].type == path_.segments[path_number].BEZIER5) {
          path_end.x = path_.segments[path_number].bezier5.p5.x;
          path_end.y = path_.segments[path_number].bezier5.p5.y;
     }
     double error_x = std::pow(path_end.x - current.x, 2);
     double error_y = std::pow(path_end.y - current.y, 2);
     double error_dist = std::sqrt(error_x + error_y);
     if (error_dist < lookaheaddist_) {
          path_number ++;
          if (path_number >= path_.segments.size()) {
               path_number = 0;
          }
          std::cout << getCurrentTime() << "Tracking path number: " << path_number
               << " type: " << path_.segments[path_number].type << std::endl;
     }

     // 前视距离
     double lookaheaddistance = k_ * v_ + lookaheaddist_;

     Pose target;
     // Step 1 基于前视距离和当前位置计算不同类型曲线的目标点
     if (path_.segments[path_number].type == path_.segments[path_number].LINE) {
          // 计算直线上的目标点
          calculateTargetOnLine(target, lookaheaddistance, current, path_.segments[path_number].line);
          
     } else if (path_.segments[path_number].type == path_.segments[path_number].BEZIER3) {
          // 计算bezier3上的目标点
          calculateTargetOnBezier3(target, lookaheaddistance, current, path_.segments[path_number].bezier3);

     } else if (path_.segments[path_number].type == path_.segments[path_number].BEZIER5) {
          // 计算bezier5上的目标点
          calculateTargetOnBezier5(target, lookaheaddistance, current, path_.segments[path_number].bezier5);
     }

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

     // //  安全限制
     // if (fabs(v_) > max_v_) {
     //      v_ = 0.0f;
     //      std::cout << getCurrentTime() << "Anomaly detected: v_" << std::endl;
     // }
     // if (fabs(w_) > max_w_) {
     //      w_ = 0.0f;
     //      std::cout << getCurrentTime() << "Anomaly detected: w_" << std::endl;
     // }
     // if (std::isinf(r) || std::isnan(r)) {
     //      w_ = 0.0f;
     //      v_ = 0.0f;
     // }
     std::cout << getCurrentTime() << "v: " << v_ << ", w: " << w_ << ", r: " << r << std::endl;
     sendVelocity(v_, w_);

     std::cout << std::endl;
}

void PurePursuit::sendVelocity(const double v, const double w)
{
     auto msg = geometry_msgs::msg::Twist();
     msg.linear.x = v;
     msg.angular.z = w;
     vel_->publish(msg);
}

void PurePursuit::pathSubCallback(const algorithm_msgs::msg::Path::SharedPtr msg)
{
     path_ = *msg;
     if (is_path_received_) {
          return;
     }
     for (size_t i = 0; i < path_.segments.size(); i++) {
          if (path_.segments[i].type == path_.segments[i].LINE) {
               std::cout << "Path " << i << " : " 
                    << "start[" << path_.segments[i].line.p0.x << ", " << path_.segments[i].line.p0.y << "] "
                    << "end[" << path_.segments[i].line.p1.x << ", " << path_.segments[i].line.p1.y << "]" << std::endl;
          } else if (path_.segments[i].type == path_.segments[i].BEZIER3) {
               std::cout << "Path " << i << " : " 
                    << "p0[" << path_.segments[i].bezier3.p0.x << ", " << path_.segments[i].bezier3.p0.y << "] "
                    << "p1[" << path_.segments[i].bezier3.p1.x << ", " << path_.segments[i].bezier3.p1.y << "] "
                    << "p2[" << path_.segments[i].bezier3.p2.x << ", " << path_.segments[i].bezier3.p2.y << "] "
                    << "p3[" << path_.segments[i].bezier3.p3.x << ", " << path_.segments[i].bezier3.p3.y << "]" << std::endl;
          } else if (path_.segments[i].type == path_.segments[i].BEZIER5) {
               std::cout << "Path " << i << " : " 
                    << "p0[" << path_.segments[i].bezier5.p0.x << ", " << path_.segments[i].bezier5.p0.y << "] "
                    << "p1[" << path_.segments[i].bezier5.p1.x << ", " << path_.segments[i].bezier5.p1.y << "] "
                    << "p2[" << path_.segments[i].bezier5.p2.x << ", " << path_.segments[i].bezier5.p2.y << "] "
                    << "p3[" << path_.segments[i].bezier5.p3.x << ", " << path_.segments[i].bezier5.p3.y << "] "
                    << "p4[" << path_.segments[i].bezier5.p4.x << ", " << path_.segments[i].bezier5.p4.y << "] "
                    << "p5[" << path_.segments[i].bezier5.p5.x << ", " << path_.segments[i].bezier5.p5.y << "]" << std::endl;
          }
     }
     is_path_received_ = true;
}

void PurePursuit::calculateTargetOnLine(Pose & target, const double lookahead, const Pose & current, const algorithm_msgs::msg::Line line)
{
     // Step 1 计算最近点
     Pose closest;
     Vector ps_pc; // 直线起点至当前位置的向量
     Vector ps_pe; // 直线起点至直线终点的向量
     ps_pc.vx = current.x - line.p0.x;
     ps_pc.vy = current.y - line.p0.y;
     ps_pe.vx = line.p1.x - line.p0.x;
     ps_pe.vy = line.p1.y - line.p0.y;
     double ps_pe_square_length = std::pow(ps_pe.vx, 2) + std::pow(ps_pe.vy, 2);
     double t = (ps_pc.vx * ps_pe.vx + ps_pc.vy * ps_pe.vy) / ps_pe_square_length;
     if (t > 1.0f) {
          t = 1.0f;
     } else if (t < 0.0f) {
          t = 0.0f;
     }
     closest.x = line.p0.x + t * ps_pe.vx;
     closest.y = line.p0.y + t * ps_pe.vy;

     // Step 2 计算单位向量
     double line_length = std::sqrt(ps_pe_square_length);
     Vector unit_vector_line;
     unit_vector_line.vx = ps_pe.vx / line_length;
     unit_vector_line.vy = ps_pe.vy / line_length;

     // Step 3 当前点到直线的最近的点 + 前视距离 * 单位向量
     target.x = closest.x + lookahead * unit_vector_line.vx;
     target.y = closest.y + lookahead * unit_vector_line.vy;

     std::cout << getCurrentTime() << "Current: [x " << current.x << ", y " << current.y << "]"
          << " target: [x " << target.x << ", y " << target.y << "]"
          << " closest: [x " << closest.x << ", y " << closest.y << "]" << std::endl;
}

void PurePursuit::calculateTargetOnBezier3(Pose & target, const double lookahead, const Pose & current, const algorithm_msgs::msg::Bezier3 bezier3) 
{
     // 贝塞尔曲线控制点
     auto & P0 = bezier3.p0;
     auto & P1 = bezier3.p1;
     auto & P2 = bezier3.p2;
     auto & P3 = bezier3.p3;

     // 用于找到最接近lookahead距离的点
     Pose closestPoint;
     double closestDistance = std::numeric_limits<double>::max();

     // 迭代t从0到1,寻找与lookahead距离最接近的点
     int steps = 100; // 可调整步数
     for (int i = 0; i <= steps; ++i) {
          double t = static_cast<double>(i) / steps;

          // 计算贝塞尔曲线上的点
          double x = std::pow(1 - t, 3) * P0.x +
                    3 * std::pow(1 - t, 2) * t * P1.x +
                    3 * (1 - t) * std::pow(t, 2) * P2.x +
                    std::pow(t, 3) * P3.x;

          double y = std::pow(1 - t, 3) * P0.y +
                    3 * std::pow(1 - t, 2) * t * P1.y +
                    3 * (1 - t) * std::pow(t, 2) * P2.y +
                    std::pow(t, 3) * P3.y;

          Pose pointOnBezier = {x, y, 0.0}; // 假设Pose的yaw坐标为0

          double dist = distance(current, pointOnBezier);

          // 找到距离current等于lookahead的点
          if (std::abs(dist - lookahead) < closestDistance) {
               closestDistance = std::abs(dist - lookahead);
               closestPoint = pointOnBezier;
          }
     }
     std::cout << getCurrentTime() << "Current: [x " << current.x << ", y " << current.y << "]"
          << " target: [x " << target.x << ", y " << target.y << "]"
          << " closest: [x " << closestPoint.x << ", y " << closestPoint.y << "]" << std::endl;
     target = closestPoint;
}

void PurePursuit::calculateTargetOnBezier5(Pose & target, const double lookahead, const Pose & current, const algorithm_msgs::msg::Bezier5 bezier5) 
{
     // 贝塞尔曲线控制点
     auto & P0 = bezier5.p0;
     auto & P1 = bezier5.p1;
     auto & P2 = bezier5.p2;
     auto & P3 = bezier5.p3;
     auto & P4 = bezier5.p4;
     auto & P5 = bezier5.p5;

     // 用于找到最接近lookahead距离的点
     Pose closestPoint;
     double closestDistance = std::numeric_limits<double>::max();

     // 迭代t从0到1,寻找与lookahead距离最接近的点
     int steps = 100; // 可调整步数
     for (int i = 0; i <= steps; ++i) {
          double t = static_cast<double>(i) / steps;

          // 计算贝塞尔曲线上的点
          double x = std::pow(1 - t, 5) * P0.x +
                    5 * std::pow(1 - t, 4) * t * P1.x +
                    10 * std::pow(1 - t, 3) * std::pow(t, 2) * P2.x +
                    10 * std::pow(1 - t, 2) * std::pow(t, 3) * P3.x +
                    5 * (1 - t) * std::pow(t, 4) * P4.x +
                    std::pow(t, 5) * P5.x;

          double y = std::pow(1 - t, 5) * P0.y +
                    5 * std::pow(1 - t, 4) * t * P1.y +
                    10 * std::pow(1 - t, 3) * std::pow(t, 2) * P2.y +
                    10 * std::pow(1 - t, 2) * std::pow(t, 3) * P3.y +
                    5 * (1 - t) * std::pow(t, 4) * P4.y +
                    std::pow(t, 5) * P5.y;

          Pose pointOnBezier = {x, y, 0.0}; // 假设Pose的yaw坐标为0

          double dist = distance(current, pointOnBezier);

          // 找到距离current等于lookahead的点
          if (std::abs(dist - lookahead) < closestDistance) {
               closestDistance = std::abs(dist - lookahead);
               closestPoint = pointOnBezier;
          }
     }
     std::cout << getCurrentTime() << "Current: [x " << current.x << ", y " << current.y << "]"
          << " target: [x " << target.x << ", y " << target.y << "]"
          << " closest: [x " << closestPoint.x << ", y " << closestPoint.y << "]" << std::endl;
     target = closestPoint;
}


}  // namespace pure_pursuit
