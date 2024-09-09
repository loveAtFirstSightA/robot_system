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

#ifndef DATA_RECORDER__DATA_RECORDER_HPP_
#define DATA_RECORDER__DATA_RECORDER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "spdlog/spdlog.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace data_recorder
{
class DataRecorder : public rclcpp::Node
{
public:
    DataRecorder();
    ~DataRecorder();

private:
    bool getCurrentPose(geometry_msgs::msg::Vector3 & pose);
    // void timer_callback();
    void odom_sub_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    // void odom_data_sub_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    // void imu_sub_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
    // void imu_data_sub_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void vel_sub_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    nav_msgs::msg::Odometry odom_pose_;
    // rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_data_sub_;
    // nav_msgs::msg::Odometry odom_data_pose_;
    // rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    // sensor_msgs::msg::Imu imu_;
    // rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_data_sub_;
    // sensor_msgs::msg::Imu imu_data_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub_;
    geometry_msgs::msg::Twist vel_;

    // rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::Vector3 pose_;

private:
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    // std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    geometry_msgs::msg::Vector3 tf_pose_;

};
}  // namespace data_recorder
#endif  // DATA_RECORDER__DATA_RECORDER_HPP_
