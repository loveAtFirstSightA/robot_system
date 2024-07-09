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

#ifndef MODEL_PREDICT_CONTROL__MODEL_PREDICT_CONTROL_HPP_
#define MODEL_PREDICT_CONTROL__MODEL_PREDICT_CONTROL_HPP_

#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "algorithm_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "model_predict_control/logger.hpp"
#include "model_predict_control/common.hpp"

namespace model_predict_control
{
class ModelPredictControl : public rclcpp::Node
{
public:
    ModelPredictControl();
    ~ModelPredictControl();

private:
    void initFirstValue();
    void currentPoseCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg);
    void sendVelocity(const double v, const double w);
    void pathSubCallback(const algorithm_msgs::msg::Path::SharedPtr msg);
    // MPC function
    void robot_model(double x, double y, double theta, double v, double omega, double T, double &x_next, double &y_next, double &theta_next);
    void tracking_error(double x, double y, double theta, double x_ref, double y_ref, double theta_ref, double &e_y, double &e_theta);
    double mpc_cost(const std::vector<double> &u, double x, double y, double theta, double x_ref, double y_ref, double theta_ref, double T, double Q_e_y, double Q_e_theta, double R_v, double R_omega);
    std::vector<double> optimize(double x, double y, double theta, double x_ref, double y_ref, double theta_ref, double T, double Q_e_y, double Q_e_theta, double R_v, double R_omega);

    // 计算路径切线的航向角度
    void calculatePathHeadingOnLine(double & heading, /*const Pose & current,*/ const algorithm_msgs::msg::Line & line);
    void calculatePathHeadingOnBezier3(double & heading, const Pose & current, const algorithm_msgs::msg::Bezier3 & bezier3);
    void calculatePathHeadingOnBezier5(double & heading, const Pose & current, const algorithm_msgs::msg::Bezier5 & bezier5);
    // 计算距离当前点最近的点
    void calculateClosestPointOnLine(Pose & closest, const Pose & current, const algorithm_msgs::msg::Line & line);
    void calculateClosestPointOnBezier3(Pose & closest, const Pose & current, const algorithm_msgs::msg::Bezier3 & bezier3);
    void calculateClosestPointOnBezier5(Pose & closest, const Pose & current, const algorithm_msgs::msg::Bezier5 & bezier5);
    
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr current_pose_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_;
    rclcpp::Subscription<algorithm_msgs::msg::Path>::SharedPtr path_sub_;
    double v_;
    double w_;
    bool is_path_received_{false};
    algorithm_msgs::msg::Path path_;


};
}  // namespace model_predict_control
#endif  // MODEL_PREDICT_CONTROL__MODEL_PREDICT_CONTROL_HPP_
