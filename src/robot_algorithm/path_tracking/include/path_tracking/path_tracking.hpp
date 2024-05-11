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

#ifndef PATH_TRACKING__PATH_TRACKING_HPP_
#define PATH_TRACKING__PATH_TRACKING_HPP_

#include <vector>
#include "path_tracking/math_library/math_library.hpp"
#include "path_tracking/pure_pursuit/pure_pursuit.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace path_tracking
{
class PathTracking : public rclcpp::Node
{
public:
    PathTracking();
    ~PathTracking();

private:
    void initSubPub();
    void currentPoseSubCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg);
    void sendVelocity(double v, double w);
    void initTimer();
    void timerCallback();

    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr current_pose_sub_;
    geometry_msgs::msg::Vector3Stamped current_pose_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

private:
    // test line start[1.76, 0.00] end[1.76, -9.0]
    geometry_msgs::msg::Vector3 line_s_;
    geometry_msgs::msg::Vector3 line_e_;


};
}  // namespace path_tracking
#endif  // PATH_TRACKING__PATH_TRACKING_HPP_
