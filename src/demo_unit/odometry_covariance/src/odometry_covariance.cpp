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

#include "odometry_covariance/odometry_covariance.hpp"

namespace odometry_covariance
{
OdometryCovariance::OdometryCovariance() : Node("odometry_covariance")
{
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        // "odom_data",
        "odom",
        10,
        std::bind(&OdometryCovariance::odomSubCallback, this, std::placeholders::_1));
}

OdometryCovariance::~OdometryCovariance() {}

void OdometryCovariance::odomSubCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // Store odometry information
    odom_ = *msg;
    // Log odometry information
    spdlog::info("odom - frame_id: {}, child_frame_id: {}, position - x: {:.4f}, y: {:.4f}, z: {:.4f}, "
        "orientation - x: {:.4f}, y: {:.4f}, z: {:.4f}, w: {:.4f}, "
        "linear velocity - x: {:.4f}, y: {:.4f}, z: {:.4f}, "
        "angular velocity - x: {:.4f}, y: {:.4f}, z: {:.4f}",
        odom_.header.frame_id,
        odom_.child_frame_id,
        odom_.pose.pose.position.x,
        odom_.pose.pose.position.y,
        odom_.pose.pose.position.z,
        odom_.pose.pose.orientation.x,
        odom_.pose.pose.orientation.y,
        odom_.pose.pose.orientation.z,
        odom_.pose.pose.orientation.w,
        odom_.twist.twist.linear.x,
        odom_.twist.twist.linear.y,
        odom_.twist.twist.linear.z,
        odom_.twist.twist.angular.x,
        odom_.twist.twist.angular.y,
        odom_.twist.twist.angular.z);

    // Log pose covariance
    std::ostringstream pose_cov;
    pose_cov << "pose covariance: [";
    for (size_t i = 0; i < 36; ++i) {
        pose_cov << odom_.pose.covariance[i];
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
        twist_cov << odom_.twist.covariance[i];
        if (i < 35) {
            twist_cov << ", ";
        }
    }
    twist_cov << "]";
    spdlog::info("{}", twist_cov.str());
    spdlog::info("\n");
}

/*
gazebo仿真环境下里程计的信息
[2024-07-08 11:30:00]  odom - frame_id: odom, child_frame_id: base_footprint, position - x: 1.00747, y: 3.99456, z: 0.056219, orientation - x: -0.00152774, y: 0.00152861, z: 0.706853, w: 0.707357, linear velocity - x: -7.29832e-05, y: 3.86541e-05, z: 0, angular velocity - x: 0, y: 0, z: 1.02255e-06
pose covariance: [1e-05, 0, 0, 0, 0, 0, 0, 1e-05, 0, 0, 0, 0, 0, 0, 1e+12, 0, 0, 0, 0, 0, 0, 1e+12, 0, 0, 0, 0, 0, 0, 1e+12, 0, 0, 0, 0, 0, 0, 0.001]
twist covariance: [1e-05, 0, 0, 0, 0, 0, 0, 1e-05, 0, 0, 0, 0, 0, 0, 1e+12, 0, 0, 0, 0, 0, 0, 1e+12, 0, 0, 0, 0, 0, 0, 1e+12, 0, 0, 0, 0, 0, 0, 0.001]
*/

}  // namespace odometry_covariance
