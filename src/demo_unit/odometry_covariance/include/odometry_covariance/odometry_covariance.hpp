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

#ifndef ODOMETRY_COVARIANCE__ODOMETRY_COVARIANCE_HPP_
#define ODOMETRY_COVARIANCE__ODOMETRY_COVARIANCE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace odometry_covariance
{
class OdometryCovariance : public rclcpp::Node
{
public:
    OdometryCovariance();
    ~OdometryCovariance();

private:
    void odomSubCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    nav_msgs::msg::Odometry odom_;
};
}  // namespace odometry_covariance
#endif  // ODOMETRY_COVARIANCE__ODOMETRY_COVARIANCE_HPP_
