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

#include "sensor_data_info/sensor_data_info.hpp"

namespace sensor_data_info
{
SensorDataInfo::SensorDataInfo() : Node("sensor_data_info")
{
    odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&SensorDataInfo::OdometrySubCallback, this, std::placeholders::_1));
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu", 10, std::bind(&SensorDataInfo::ImuSubCallback, this, std::placeholders::_1));

}

SensorDataInfo::~SensorDataInfo() {}

void SensorDataInfo::OdometrySubCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    spdlog::info("\n");
    odometry_ = *msg;

    // Log odometry information
    spdlog::info("odom - frame_id: {}, child_frame_id: {}, position - x: {:.4f}, y: {:.4f}, z: {:.4f}, "
        "orientation - x: {:.4f}, y: {:.4f}, z: {:.4f}, w: {:.4f}, "
        "linear velocity - x: {:.4f}, y: {:.4f}, z: {:.4f}, "
        "angular velocity - x: {:.4f}, y: {:.4f}, z: {:.4f}",
        odometry_.header.frame_id,
        odometry_.child_frame_id,
        odometry_.pose.pose.position.x,
        odometry_.pose.pose.position.y,
        odometry_.pose.pose.position.z,
        odometry_.pose.pose.orientation.x,
        odometry_.pose.pose.orientation.y,
        odometry_.pose.pose.orientation.z,
        odometry_.pose.pose.orientation.w,
        odometry_.twist.twist.linear.x,
        odometry_.twist.twist.linear.y,
        odometry_.twist.twist.linear.z,
        odometry_.twist.twist.angular.x,
        odometry_.twist.twist.angular.y,
        odometry_.twist.twist.angular.z);

    // Log pose covariance
    std::ostringstream pose_cov;
    pose_cov << "pose covariance: [";
    for (size_t i = 0; i < 36; ++i) {
        pose_cov << odometry_.pose.covariance[i];
        if (i < 35) {
            pose_cov << ", ";
        }
    }
    pose_cov << "]";
    spdlog::info("{}", pose_cov.str());

    // Log twist covariance
    std::ostringstream twist_cov;
    twist_cov << "twist covariance: [";
    for (size_t i = 0; i < 36; ++i) {
        twist_cov << odometry_.twist.covariance[i];
        if (i < 35) {
            twist_cov << ", ";
        }
    }
    twist_cov << "]";
    spdlog::info("{}", twist_cov.str());
    spdlog::info("\n");

    // Log imu information
    spdlog::info("imu - frame_id: {}, yaw: {:.4f}, linear_acceleration - x: {:.4f}, y: {:.4f}, z: {:.4f}, "
        "angular_velocity - x: {:.4f}, y: {:.4f}, z: {:.4f}",
        imu_.header.frame_id,
        tf2::getYaw(imu_.orientation),
        imu_.linear_acceleration.x,
        imu_.linear_acceleration.y,
        imu_.linear_acceleration.z,
        imu_.angular_velocity.x,
        imu_.angular_velocity.y,
        imu_.angular_velocity.z);

    // Log orientation_covariance
    std::ostringstream orientation_covariance;
    orientation_covariance << "orientation_covariance: [";
    for (size_t i = 0; i < imu_.orientation_covariance.size(); ++i) {
        orientation_covariance << odometry_.pose.covariance[i];
        if (i < (imu_.orientation_covariance.size() - 1)) {
            orientation_covariance << ", ";
        }
    }
    orientation_covariance << "]";
    spdlog::info("{}", orientation_covariance.str());

    // Log linear_acceleration_covariance
    std::ostringstream linear_acceleration_covariance;
    linear_acceleration_covariance << "linear_acceleration_covariance: [";
    for (size_t i = 0; i < imu_.linear_acceleration_covariance.size(); ++i) {
        linear_acceleration_covariance << odometry_.twist.covariance[i];
        if (i < (imu_.linear_acceleration_covariance.size() - 1)) {
            linear_acceleration_covariance << ", ";
        }
    }
    linear_acceleration_covariance << "]";
    spdlog::info("{}", linear_acceleration_covariance.str());

    // Log angular_velocity_covariance
    std::ostringstream angular_velocity_covariance;
    angular_velocity_covariance << "angular_velocity_covariance: [";
    for (size_t i = 0; i < imu_.angular_velocity_covariance.size(); ++i) {
        angular_velocity_covariance << odometry_.twist.covariance[i];
        if (i < (imu_.angular_velocity_covariance.size() - 1)) {
            angular_velocity_covariance << ", ";
        }
    }
    angular_velocity_covariance << "]";
    spdlog::info("{}", angular_velocity_covariance.str());
    
}

void SensorDataInfo::ImuSubCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    imu_ = *msg;
}



}  // namespace sensor_data_info
