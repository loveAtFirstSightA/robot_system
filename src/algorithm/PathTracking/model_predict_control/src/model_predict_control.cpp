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

#include "model_predict_control/model_predict_control.hpp"

namespace model_predict_control
{
ModelPredictControl::ModelPredictControl() : Node("model_predict_control")
{
     initFirstValue();
     current_pose_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
          "estimate_pose",
          rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
          std::bind(&ModelPredictControl::currentPoseCallback, this, std::placeholders::_1));
     path_sub_ = this->create_subscription<algorithm_msgs::msg::Path>(
          "algorithm_path",
          rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
          std::bind(&ModelPredictControl::pathSubCallback, this, std::placeholders::_1));
     vel_ = this->create_publisher<geometry_msgs::msg::Twist>(
          "cmd_vel",
          10);
}

ModelPredictControl::~ModelPredictControl() 
{
     sendVelocity(0.0f, 0.0f);
}

void ModelPredictControl::initFirstValue()
{
     // this->v_ = 0.6f;
}

// 机器人运动学模型
void ModelPredictControl::robot_model(double x, double y, double theta, double v, double omega, double T, double &x_next, double &y_next, double &theta_next) 
{
     x_next = x + T * v * std::cos(theta);
     y_next = y + T * v * std::sin(theta);
     theta_next = theta + T * omega;
}

// 路径跟踪误差
void ModelPredictControl::tracking_error(double x, double y, double theta, double x_ref, double y_ref, double theta_ref, double &e_y, double &e_theta)
{
     e_y = (y - y_ref) * cos(theta_ref) - (x - x_ref) * sin(theta_ref);
     e_theta = theta - theta_ref;
}

// MPC优化目标函数
double ModelPredictControl::mpc_cost(const std::vector<double> &u, double x, double y, double theta, double x_ref, double y_ref, double theta_ref, double T, double Q_e_y, double Q_e_theta, double R_v, double R_omega)
{
     double v = u[0];
     double omega = u[1];
     double x_next, y_next, theta_next;
     robot_model(x, y, theta, v, omega, T, x_next, y_next, theta_next);
     double e_y, e_theta;
     tracking_error(x_next, y_next, theta_next, x_ref, y_ref, theta_ref, e_y, e_theta);
     double cost = Q_e_y * e_y * e_y + Q_e_theta * e_theta * e_theta + R_v * v * v + R_omega * omega * omega;
     return cost;
}

// 优化求解
// 增加对控制输入的搜索范围
std::vector<double> ModelPredictControl::optimize(double x, double y, double theta, double x_ref, double y_ref, double theta_ref, double T, double Q_e_y, double Q_e_theta, double R_v, double R_omega)
{
     std::vector<double> u = {0, 0}; // 初始控制输入
     double best_cost = std::numeric_limits<double>::max();
     std::vector<double> best_u = u;

     // 调整控制输入的搜索步长
     for (double v = -1.0; v <= 1.0; v += 0.01) {
          for (double omega = -M_PI / 4; omega <= M_PI / 4; omega += 0.01) {
               u[0] = v;
               u[1] = omega;
               double cost = mpc_cost(u, x, y, theta, x_ref, y_ref, theta_ref, T, Q_e_y, Q_e_theta, R_v, R_omega);
               if (cost < best_cost) {
                    best_cost = cost;
                    best_u = u;
               }
          }
     }

     return best_u;
}

void ModelPredictControl::calculatePathHeadingOnLine(double & heading, /*const Pose & current,*/ const algorithm_msgs::msg::Line & line)
{
     // 计算直线路径的航向角度
     Vector path;
     path.vx = line.p1.x - line.p0.x;
     path.vy = line.p1.y - line.p0.y;
     heading = std::atan2(path.vy, path.vx);
}

void ModelPredictControl::calculatePathHeadingOnBezier3(double & heading, const Pose & current, const algorithm_msgs::msg::Bezier3 & bezier3)
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

void ModelPredictControl::calculatePathHeadingOnBezier5(double & heading, const Pose & current, const algorithm_msgs::msg::Bezier5 & bezier5)
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

void ModelPredictControl::calculateClosestPointOnLine(Pose & closest, const Pose & current, const algorithm_msgs::msg::Line & line) 
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


void ModelPredictControl::calculateClosestPointOnBezier3(Pose & closest, const Pose & current, const algorithm_msgs::msg::Bezier3 & bezier3) 
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

void ModelPredictControl::calculateClosestPointOnBezier5(Pose & closest, const Pose & current, const algorithm_msgs::msg::Bezier5 & bezier5) 
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

// main function of ModelPredictControl algorithm 
void ModelPredictControl::currentPoseCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
{
     if (!is_path_received_) {
          std::cout << getCurrentTime() << "path is empty, return" << std::endl;
          sendVelocity(0.0f, 0.0f);
          return;
     }
     Pose current;
     current.x = msg->vector.x;
     current.y = msg->vector.y;
     current.yaw = msg->vector.z;
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
     // 当前位置距离路径终点小于阈值，则切换路径
     const double change_path_threshold = 0.5f;
     if (error_dist < change_path_threshold) {
          path_number ++;
          if (path_number >= path_.segments.size()) {
               path_number = 0;
          }
          std::cout << getCurrentTime() << "Tracking path number: " << path_number
               << " type: " << path_.segments[path_number].type << std::endl;
     }

     // 计算距离路径最近的点
     Pose closest;
     if (path_.segments[path_number].type == path_.segments[path_number].LINE) {
          calculateClosestPointOnLine(closest, current, path_.segments[path_number].line);
     } else if (path_.segments[path_number].type == path_.segments[path_number].BEZIER3) {
          calculateClosestPointOnBezier3(closest, current, path_.segments[path_number].bezier3);
     } else if (path_.segments[path_number].type == path_.segments[path_number].BEZIER5) {
          calculateClosestPointOnBezier5(closest, current, path_.segments[path_number].bezier5);
     }
     // 计算路径最近点的切线的航向角度
     double path_heading;
     if (path_.segments[path_number].type == path_.segments[path_number].LINE) {
          calculatePathHeadingOnLine(path_heading, path_.segments[path_number].line);
     } else if (path_.segments[path_number].type == path_.segments[path_number].BEZIER3) {
          calculatePathHeadingOnBezier3(path_heading, current, path_.segments[path_number].bezier3);
     } else if (path_.segments[path_number].type == path_.segments[path_number].BEZIER5) {
          calculatePathHeadingOnBezier5(path_heading, current, path_.segments[path_number].bezier5);
     }

     // 初始状态和参考轨迹
     double x = current.x;
     double y = current.y;
     double theta = current.yaw;

     double x_ref = closest.x;
     double y_ref = closest.y;
     double theta_ref = path_heading; // 你可以根据实际情况调整参考yaw角

     double T = 0.05;  // 50ms
     double Q_e_y = 1, Q_e_theta = 1, R_v = 0.1, R_omega = 0.1;

     // 优化求解
     std::vector<double> optimal_control = optimize(x, y, theta, x_ref, y_ref, theta_ref, T, Q_e_y, Q_e_theta, R_v, R_omega);
     double v_opt = optimal_control[0];
     double omega_opt = optimal_control[1];
     std::cout << "target pose:[x " << x_ref << ", y " << y_ref << ", yaw: " << theta_ref << "]" 
          << ", current:[x " << current.x << ", y " << current.y << ", yaw " << current.yaw << "]" << std::endl;
     // 应用最优控制输入
     double x_next, y_next, theta_next;
     robot_model(x, y, theta, v_opt, omega_opt, T, x_next, y_next, theta_next);

     // 输出结果
     std::cout << "Optimal v: " << v_opt << std::endl;
     std::cout << "Optimal omega: " << omega_opt << std::endl;
     std::cout << "Next state -> x: " << x_next << ", y: " << y_next << ", theta: " << theta_next << std::endl;

     // 发送速度指令到机器人
     sendVelocity(v_opt, omega_opt);

     std::cout << std::endl;
}

void ModelPredictControl::sendVelocity(const double v, const double w)
{
     auto msg = geometry_msgs::msg::Twist();
     msg.linear.x = v;
     msg.angular.z = w;
     vel_->publish(msg);
}

void ModelPredictControl::pathSubCallback(const algorithm_msgs::msg::Path::SharedPtr msg)
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


}  // namespace model_predict_control
