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

#include <limits>
#include <cmath>
#include <iostream>
#include <string>
#include <cstdint>  // for uint8_t
#include "stanley_controller/stanley_controller.hpp"
#include "stanley_controller/common.hpp"

namespace stanley_controller
{
StanleyController::StanleyController() : Node("stanley_controller")
{
     updateParameters();
     initFirstValue();

     path_sub_ = this->create_subscription<algorithm_msgs::msg::Path>(
          "algorithm_path",
          rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
          std::bind(&StanleyController::pathSubCallback, this, std::placeholders::_1));
     vel_ = this->create_publisher<geometry_msgs::msg::Twist>(
          "cmd_vel",
          1);
     
     tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
     tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
     timer_ = this->create_wall_timer(
          std::chrono::milliseconds(20),
          std::bind(&StanleyController::timerCallback, this));
}

StanleyController::~StanleyController() {}

void StanleyController::updateParameters()
{
     this->declare_parameter<double>("k", 0.4f);

     k_ = this->get_parameter("k").get_value<double>();
}

void StanleyController::initFirstValue()
{
     this->v_ = 0.3f;
}

void StanleyController::sendVelocity(const double v, const double w)
{
     auto msg = geometry_msgs::msg::Twist();
     msg.linear.x = v;
     msg.angular.z = w;
     vel_->publish(msg);
}

void StanleyController::pathSubCallback(const algorithm_msgs::msg::Path::SharedPtr msg)
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

bool StanleyController::getCurrentPose(double & x, double & y, double & yaw)
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

void StanleyController::timerCallback()
{
     if (!is_path_received_) {
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
     
     const double change_path_threshold = 0.25f;
     if (error_dist < change_path_threshold) {
          path_number ++;
          if (path_number >= path_.segments.size()) {
               path_number = 0;
          }
     }
     // Step 1 假设交叉航迹误差为零，计算航向偏差角度
     // Step 1.1 计算路径的航向角 三种类型的路径
     double path_heading;
     if (path_.segments[path_number].type == path_.segments[path_number].LINE) {
          calculatePathHeadingOnLine(path_heading, path_.segments[path_number].line);
     } else if (path_.segments[path_number].type == path_.segments[path_number].BEZIER3) {
          calculatePathHeadingOnBezier3(path_heading, current, path_.segments[path_number].bezier3);
     } else if (path_.segments[path_number].type == path_.segments[path_number].BEZIER5) {
          calculatePathHeadingOnBezier5(path_heading, current, path_.segments[path_number].bezier5);
     }
     // Step 1.2 计算航向偏差
     double theta_varphi = path_heading - current.yaw;
     // 角度标准化
     theta_varphi = normalizeAngle(theta_varphi);

     // Step 2 假设航向偏差为零， 计算矫正交叉航迹航迹偏差的航向偏差角度
     // Step 2.1 计算横向偏差
     // Step 2.1.1 寻找距离路径最近的点
     Pose closest;
     if (path_.segments[path_number].type == path_.segments[path_number].LINE) {
          calculateClosestPointOnLine(closest, current, path_.segments[path_number].line);
     } else if (path_.segments[path_number].type == path_.segments[path_number].BEZIER3) {
          calculateClosestPointOnBezier3(closest, current, path_.segments[path_number].bezier3);
     } else if (path_.segments[path_number].type == path_.segments[path_number].BEZIER5) {
          calculateClosestPointOnBezier5(closest, current, path_.segments[path_number].bezier5);
     }
     // 计算路径最近点与当前点的欧氏距离
     double shortest_distance = std::sqrt(std::pow((closest.x - current.x), 2) + std::pow((closest.y - current.y), 2));
     // 确定横向偏差是在路劲的左侧还是右侧 右手坐标系 偏差在路径左侧为正 偏差在路径右侧为负
     double lateral_error_sign = calculateLateralErrorSign(current, closest, path_heading);
     double signed_shortest_distance = shortest_distance * lateral_error_sign;
     // Step 2.2 根据速度v和参数k确定d_t
     double theta_y = std::atan2(signed_shortest_distance, v_ / k_);
     // Step 2.3 计算转角 theta
     double theta = theta_varphi + theta_y;
     theta = normalizeAngle(theta);
     // 角度偏差转换成角速度目标值
     // w_ = v_ * std::tan(theta) / 0.5f;
     w_ = theta / 0.25f;
     // send velocity
     sendVelocity(v_, w_);

     std::cout << std::endl;
}

void StanleyController::calculatePathHeadingOnLine(double & heading, /*const Pose & current,*/ const algorithm_msgs::msg::Line & line)
{
     // 计算直线路径的航向角度
     Vector path;
     path.vx = line.p1.x - line.p0.x;
     path.vy = line.p1.y - line.p0.y;
     heading = std::atan2(path.vy, path.vx);
}

void StanleyController::calculatePathHeadingOnBezier3(double & heading, const Pose & current, const algorithm_msgs::msg::Bezier3 & bezier3)
{
     // 计算三阶贝塞尔曲线上距离当前位置最近的点为切点的切线角度
     // 获取贝塞尔曲线的控制点
     const auto & P0 = bezier3.p0;
     const auto & P1 = bezier3.p1;
     const auto & P2 = bezier3.p2;
     const auto & P3 = bezier3.p3;

     // 初始化一个变量 min_distance，并将其设置为 double 类型能够表示的最大值。
     // 这样做的目的是为了确保在后续的距离计算过程中，任何实际的距离值都会小于这个初始值，从而能够正确更新 min_distance。
     double min_distance = std::numeric_limits<double>::max();
     double closest_t = 0.0;

     // 迭代求解距离当前位置最近的点
     for (double t = 0; t <= 1; t += 0.01) {
          double x = std::pow(1-t, 3) * P0.x + 3 * std::pow(1-t, 2) * t * P1.x + 3 * (1-t) * std::pow(t, 2) * P2.x + std::pow(t, 3) * P3.x;
          double y = std::pow(1-t, 3) * P0.y + 3 * std::pow(1-t, 2) * t * P1.y + 3 * (1-t) * std::pow(t, 2) * P2.y + std::pow(t, 3) * P3.y;

          double distance = std::sqrt(std::pow(current.x - x, 2) + std::pow(current.y - y, 2));
          if (distance < min_distance) {
               min_distance = distance;
               closest_t = t;
          }
     }

     // 计算最近点处的切线
     double dx = -3 * std::pow(1-closest_t, 2) * P0.x + 3 * (std::pow(1-closest_t, 2) - 2 * closest_t * (1-closest_t)) * P1.x + 3 * ((1-closest_t) * 2 * closest_t - std::pow(closest_t, 2)) * P2.x + 3 * std::pow(closest_t, 2) * P3.x;
     double dy = -3 * std::pow(1-closest_t, 2) * P0.y + 3 * (std::pow(1-closest_t, 2) - 2 * closest_t * (1-closest_t)) * P1.y + 3 * ((1-closest_t) * 2 * closest_t - std::pow(closest_t, 2)) * P2.y + 3 * std::pow(closest_t, 2) * P3.y;

     // 计算切线的角度
     heading = std::atan2(dy, dx);
}

void StanleyController::calculatePathHeadingOnBezier5(double & heading, const Pose & current, const algorithm_msgs::msg::Bezier5 & bezier5)
{
     // 计算五阶贝塞尔曲线上距离当前位置最近的点为切点的切线角度
     // 获取贝塞尔曲线的控制点
     const auto & P0 = bezier5.p0;
     const auto & P1 = bezier5.p1;
     const auto & P2 = bezier5.p2;
     const auto & P3 = bezier5.p3;
     const auto & P4 = bezier5.p4;
     const auto & P5 = bezier5.p5;

     double min_distance = std::numeric_limits<double>::max();
     double closest_t = 0.0;

     // 迭代求解距离当前位置最近的点
     for (double t = 0; t <= 1; t += 0.01) {
          double x = std::pow(1-t, 5) * P0.x + 5 * std::pow(1-t, 4) * t * P1.x + 10 * std::pow(1-t, 3) * std::pow(t, 2) * P2.x
                    + 10 * std::pow(1-t, 2) * std::pow(t, 3) * P3.x + 5 * (1-t) * std::pow(t, 4) * P4.x + std::pow(t, 5) * P5.x;
          double y = std::pow(1-t, 5) * P0.y + 5 * std::pow(1-t, 4) * t * P1.y + 10 * std::pow(1-t, 3) * std::pow(t, 2) * P2.y
                    + 10 * std::pow(1-t, 2) * std::pow(t, 3) * P3.y + 5 * (1-t) * std::pow(t, 4) * P4.y + std::pow(t, 5) * P5.y;

          double distance = std::sqrt(std::pow(current.x - x, 2) + std::pow(current.y - y, 2));
          if (distance < min_distance) {
               min_distance = distance;
               closest_t = t;
          }
     }

     // 计算最近点处的切线
     double dx = -5 * std::pow(1-closest_t, 4) * P0.x + 5 * (4 * std::pow(1-closest_t, 3) * closest_t - std::pow(1-closest_t, 4)) * P1.x
               + 10 * (3 * std::pow(1-closest_t, 2) * std::pow(closest_t, 2) - 2 * std::pow(1-closest_t, 3) * closest_t) * P2.x
               + 10 * (2 * (1-closest_t) * std::pow(closest_t, 3) - 3 * std::pow(1-closest_t, 2) * std::pow(closest_t, 2)) * P3.x
               + 5 * (std::pow(closest_t, 4) - 4 * (1-closest_t) * std::pow(closest_t, 3)) * P4.x + 5 * std::pow(closest_t, 4) * P5.x;
     double dy = -5 * std::pow(1-closest_t, 4) * P0.y + 5 * (4 * std::pow(1-closest_t, 3) * closest_t - std::pow(1-closest_t, 4)) * P1.y
               + 10 * (3 * std::pow(1-closest_t, 2) * std::pow(closest_t, 2) - 2 * std::pow(1-closest_t, 3) * closest_t) * P2.y
               + 10 * (2 * (1-closest_t) * std::pow(closest_t, 3) - 3 * std::pow(1-closest_t, 2) * std::pow(closest_t, 2)) * P3.y
               + 5 * (std::pow(closest_t, 4) - 4 * (1-closest_t) * std::pow(closest_t, 3)) * P4.y + 5 * std::pow(closest_t, 4) * P5.y;

     // 计算切线的角度
     heading = std::atan2(dy, dx);
}

void StanleyController::calculateClosestPointOnLine(Pose & closest, const Pose & current, const algorithm_msgs::msg::Line & line) 
{
     // 提取线的端点
     double x1 = line.p0.x, y1 = line.p0.y;
     double x2 = line.p1.x, y2 = line.p1.y;

     // 当前点坐标
     double x0 = current.x, y0 = current.y;

     // 计算直线的向量
     double dx = x2 - x1;
     double dy = y2 - y1;

     // 计算投影因子t
     double t = ((x0 - x1) * dx + (y0 - y1) * dy) / (dx * dx + dy * dy);

     // 确保t在[0, 1]范围内
     t = std::max(0.0, std::min(1.0, t));

     // 计算最近点的坐标
     closest.x = x1 + t * dx;
     closest.y = y1 + t * dy;
}


void StanleyController::calculateClosestPointOnBezier3(Pose & closest, const Pose & current, const algorithm_msgs::msg::Bezier3 & bezier3) 
{
     const auto & P0 = bezier3.p0;
     const auto & P1 = bezier3.p1;
     const auto & P2 = bezier3.p2;
     const auto & P3 = bezier3.p3;

     double min_distance = std::numeric_limits<double>::max();
     // double closest_t = 0.0;

     for (double t = 0; t <= 1; t += 0.01) {
          double x = std::pow(1-t, 3) * P0.x + 3 * std::pow(1-t, 2) * t * P1.x + 3 * (1-t) * std::pow(t, 2) * P2.x + std::pow(t, 3) * P3.x;
          double y = std::pow(1-t, 3) * P0.y + 3 * std::pow(1-t, 2) * t * P1.y + 3 * (1-t) * std::pow(t, 2) * P2.y + std::pow(t, 3) * P3.y;

          double distance = std::sqrt(std::pow(current.x - x, 2) + std::pow(current.y - y, 2));
          if (distance < min_distance) {
               min_distance = distance;
               // closest_t = t;
               closest.x = x;
               closest.y = y;
          }
     }
}

void StanleyController::calculateClosestPointOnBezier5(Pose & closest, const Pose & current, const algorithm_msgs::msg::Bezier5 & bezier5) 
{
     const auto & P0 = bezier5.p0;
     const auto & P1 = bezier5.p1;
     const auto & P2 = bezier5.p2;
     const auto & P3 = bezier5.p3;
     const auto & P4 = bezier5.p4;
     const auto & P5 = bezier5.p5;

     double min_distance = std::numeric_limits<double>::max();
     // double closest_t = 0.0;

     for (double t = 0; t <= 1; t += 0.01) {
          double x = std::pow(1-t, 5) * P0.x + 5 * std::pow(1-t, 4) * t * P1.x + 10 * std::pow(1-t, 3) * std::pow(t, 2) * P2.x
                    + 10 * std::pow(1-t, 2) * std::pow(t, 3) * P3.x + 5 * (1-t) * std::pow(t, 4) * P4.x + std::pow(t, 5) * P5.x;
          double y = std::pow(1-t, 5) * P0.y + 5 * std::pow(1-t, 4) * t * P1.y + 10 * std::pow(1-t, 3) * std::pow(t, 2) * P2.y
                    + 10 * std::pow(1-t, 2) * std::pow(t, 3) * P3.y + 5 * (1-t) * std::pow(t, 4) * P4.y + std::pow(t, 5) * P5.y;

          double distance = std::sqrt(std::pow(current.x - x, 2) + std::pow(current.y - y, 2));
          if (distance < min_distance) {
               min_distance = distance;
               // closest_t = t;
               closest.x = x;
               closest.y = y;
          }
     }
}

double StanleyController::calculateLateralErrorSign(const Pose& current, const Pose& closest, double path_heading)
{
    // 计算路径的法线向量
    double normal_x = -std::sin(path_heading);
    double normal_y = std::cos(path_heading);
    
    // 计算当前点到路径最近点的向量
    double vec_x = current.x - closest.x;
    double vec_y = current.y - closest.y;
    
    // 计算路径法线向量和当前点到路径最近点向量的点积
    double dot_product = normal_x * vec_x + normal_y * vec_y;
    
    // 如果点积为负，则当前点在路径右侧；为正，则在左侧
    return (dot_product < 0) ? 1.0 : -1.0;
}


}  // namespace stanley_controller
