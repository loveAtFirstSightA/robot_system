/*
 Copyright 2024 Google LLC

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

#ifndef SENSOR_DATA_INFO__SENSOR_DATA_INFO_HPP_
#define SENSOR_DATA_INFO__SENSOR_DATA_INFO_HPP_

#include "rclcpp/rclcpp.hpp"
#include "spdlog/spdlog.h"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace sensor_data_info
{
class SensorDataInfo : public rclcpp::Node
{
public:
    SensorDataInfo();
    ~SensorDataInfo();

private:
    void OdometrySubCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void ImuSubCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    nav_msgs::msg::Odometry odometry_;
    sensor_msgs::msg::Imu imu_;
};
}  // namespace sensor_data_info
#endif  // SENSOR_DATA_INFO__SENSOR_DATA_INFO_HPP_
