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

#ifndef STANLEY_CONTROLLER__STANLEY_CONTROLLER_HPP_
#define STANLEY_CONTROLLER__STANLEY_CONTROLLER_HPP_

#include "stanley_controller/common.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "algorithm_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "tf2/utils.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace stanley_controller
{
class StanleyController : public rclcpp::Node
{
public:
    StanleyController();
    ~StanleyController();

private:
    void updateParameters();
    void initFirstValue();
    void sendVelocity(const double v, const double w);
    void pathSubCallback(const algorithm_msgs::msg::Path::SharedPtr msg);
    // 计算路径切线的航向角度
    void calculatePathHeadingOnLine(double & heading, /*const Pose & current,*/ const algorithm_msgs::msg::Line & line);
    void calculatePathHeadingOnBezier3(double & heading, const Pose & current, const algorithm_msgs::msg::Bezier3 & bezier3);
    void calculatePathHeadingOnBezier5(double & heading, const Pose & current, const algorithm_msgs::msg::Bezier5 & bezier5);
    // 计算距离当前点最近的点
    void calculateClosestPointOnLine(Pose & closest, const Pose & current, const algorithm_msgs::msg::Line & line);
    void calculateClosestPointOnBezier3(Pose & closest, const Pose & current, const algorithm_msgs::msg::Bezier3 & bezier3);
    void calculateClosestPointOnBezier5(Pose & closest, const Pose & current, const algorithm_msgs::msg::Bezier5 & bezier5);
    double calculateLateralErrorSign(const Pose& current, const Pose& closest, double path_heading);

    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr current_pose_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_;
    rclcpp::Subscription<algorithm_msgs::msg::Path>::SharedPtr path_sub_;
    bool is_path_received_{false};
    algorithm_msgs::msg::Path path_;

    double v_;
    double w_;

    // algorithm parameters
    double k_;

private:
    void timerCallback();
    bool getCurrentPose(double & x, double & y, double & yaw);

    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

};
}  // namespace stanley_controller
#endif  // STANLEY_CONTROLLER__STANLEY_CONTROLLER_HPP_
