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

#include <memory>
#include <string>
#include <cmath>
#include <iostream>
#include <chrono>
#include <ctime>
#include <iomanip>
#include "data_recorder/data_recorder.hpp"
#include "tf2/utils.h"
#include "data_recorder/logger.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace data_recorder
{
DataRecorder::DataRecorder() : Node("data_recorder")
{
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(15),
        std::bind(&DataRecorder::timer_callback, this));
    // estimate_pose_sub_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
    //     "estimate_pose",
    //     rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    //     std::bind(&DataRecorder::estimate_pose_sub_callback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom",
        100,
        std::bind(&DataRecorder::odom_sub_callback, this, std::placeholders::_1));
    odom_data_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom_data",
        100,
        std::bind(&DataRecorder::odom_data_sub_callback, this, std::placeholders::_1));
    // imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    //     "/imu",
    //     100,
    //     std::bind(&DataRecorder::imu_sub_callback, this, std::placeholders::_1));
    // imu_data_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    //     "/imu_data",
    //     100,
    //     std::bind(&DataRecorder::imu_data_sub_callback, this, std::placeholders::_1));
    vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel",
        100,
        std::bind(&DataRecorder::vel_sub_callback, this, std::placeholders::_1));
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    // tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(shared_from_this());
}

DataRecorder::~DataRecorder() {}

void DataRecorder::timer_callback()
{
    //  std::cout << getCurrentTime() << "estimate_pose: [x " << estimate_pose_.vector.x << ", y " << estimate_pose_.vector.y << ", yaw " << estimate_pose_.vector.z << "] "
    //       << "odom_data: [x " << odom_data_pose_.pose.pose.position.x << ", y " << odom_data_pose_.pose.pose.position.y << ", yaw: " << tf2::getYaw(odom_data_pose_.pose.pose.orientation) << "] "
    //       << "odom: [x " << odom_pose_.pose.pose.position.x << ", y " << odom_pose_.pose.pose.position.y << ", yaw " << tf2::getYaw(odom_pose_.pose.pose.orientation) << "] "
    //       << "imu_data: [l_acc_x " << imu_data_.linear_acceleration.x << ", l_acc_y " << imu_data_.linear_acceleration.x << ", l_acc_z " << imu_data_.linear_acceleration.z << "]"
    //       << "[a_acc_x " << imu_data_.angular_velocity.x << ", a_acc_y " << imu_data_.angular_velocity.y << ", a_acc_z " << imu_data_.angular_velocity.z << "] imu_DataRecorder_yaw: " << tf2::getYaw(imu_data_.orientation) << ", "
    //       << "imu: [l_acc_x " << imu_.linear_acceleration.x << ", l_acc_y " << imu_.linear_acceleration.x << ", l_acc_z " << imu_.linear_acceleration.z << "]"
    //       << "[a_acc_x " << imu_.angular_velocity.x << ", a_acc_y " << imu_.angular_velocity.y << ", a_acc_z " << imu_.angular_velocity.z << "] imu_yaw: " << tf2::getYaw(imu_.orientation) << ", "
    //       << "cmd_vel: [v " << vel_.linear.x << ", w " << vel_.angular.z << "]" << std::endl;

    std::string source_link = "base_footprint";
    std::string target_link = "map";
    geometry_msgs::msg::TransformStamped tf;
    try {
        tf = tf_buffer_->lookupTransform(target_link, source_link, tf2::TimePointZero);
        tf_pose_.x = tf.transform.translation.x;
        tf_pose_.y = tf.transform.translation.y;
        tf_pose_.z = tf2::getYaw(tf.transform.rotation);
    } catch(const std::exception& e) {
        // std::cerr << getCurrentTime() << " - " << e.what() << std::endl;
    }
}

// void DataRecorder::estimate_pose_sub_callback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
// {
//      this->estimate_pose_ = *msg;
// }

void DataRecorder::odom_sub_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    this->odom_pose_ = *msg;
    std::cout << getCurrentTime() << "odom: [x " << odom_pose_.pose.pose.position.x << ", y " << odom_pose_.pose.pose.position.y << ", yaw " << tf2::getYaw(odom_pose_.pose.pose.orientation) << "] "
        // << "imu_data: [l_acc_x " << imu_data_.linear_acceleration.x << ", l_acc_y " << imu_data_.linear_acceleration.x << ", l_acc_z " << imu_data_.linear_acceleration.z << "]"
        // << "[a_acc_x " << imu_data_.angular_velocity.x << ", a_acc_y " << imu_data_.angular_velocity.y << ", a_acc_z " << imu_data_.angular_velocity.z << "] imu_DataRecorder_yaw: " << tf2::getYaw(imu_data_.orientation) << ", "
        // << "imu: [l_acc_x " << imu_.linear_acceleration.x << ", l_acc_y " << imu_.linear_acceleration.x << ", l_acc_z " << imu_.linear_acceleration.z << "]"
        // << "[a_acc_x " << imu_.angular_velocity.x << ", a_acc_y " << imu_.angular_velocity.y << ", a_acc_z " << imu_.angular_velocity.z << "] imu_yaw: " << tf2::getYaw(imu_.orientation) << ", "
        << "cmd_vel: [v " << vel_.linear.x << ", w " << vel_.angular.z << "] "
        << "estimate_pose: [x " << tf_pose_.x << ", y " << tf_pose_.y << ", yaw " << tf_pose_.z << "] "
        << "odom_data: [x " << odom_data_pose_.pose.pose.position.x << ", y " << odom_data_pose_.pose.pose.position.y << ", yaw " << tf2::getYaw(odom_data_pose_.pose.pose.orientation) << "] " << std::endl;
}

void DataRecorder::odom_data_sub_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    this->odom_data_pose_ = *msg;
    // real time system
    // std::cout << getCurrentTime() << "odom_data: [x " << odom_data_pose_.pose.pose.position.x << ", y " << odom_data_pose_.pose.pose.position.y << ", yaw " << tf2::getYaw(odom_data_pose_.pose.pose.orientation) << "] "
    //     // << "imu_data: [l_acc_x " << imu_data_.linear_acceleration.x << ", l_acc_y " << imu_data_.linear_acceleration.x << ", l_acc_z " << imu_data_.linear_acceleration.z << "]"
    //     // << "[a_acc_x " << imu_data_.angular_velocity.x << ", a_acc_y " << imu_data_.angular_velocity.y << ", a_acc_z " << imu_data_.angular_velocity.z << "] imu_DataRecorder_yaw: " << tf2::getYaw(imu_data_.orientation) << ", "
    //     // << "imu: [l_acc_x " << imu_.linear_acceleration.x << ", l_acc_y " << imu_.linear_acceleration.x << ", l_acc_z " << imu_.linear_acceleration.z << "]"
    //     // << "[a_acc_x " << imu_.angular_velocity.x << ", a_acc_y " << imu_.angular_velocity.y << ", a_acc_z " << imu_.angular_velocity.z << "] imu_yaw: " << tf2::getYaw(imu_.orientation) << ", "
    //     << "cmd_vel: [v " << vel_.linear.x << ", w " << vel_.angular.z << "]"
    //     << " estimate_pose: [x " << tf_pose_.x << ", y " << tf_pose_.y << ", yaw " << tf_pose_.z << "]" << std::endl;
    
    // // bag package
    // // 模拟 odom_data_pose_.header.stamp.sec 和 odom_data_pose_.header.stamp.nanosec 的值
    // std::time_t sec = odom_data_pose_.header.stamp.sec;
    // std::uint32_t nanosec = odom_data_pose_.header.stamp.nanosec;

    // // 将秒数转换为UTC时间
    // std::chrono::system_clock::time_point tp = std::chrono::system_clock::from_time_t(sec);
    // tp += std::chrono::nanoseconds(nanosec);

    // // 转换为 time_t
    // std::time_t tt = std::chrono::system_clock::to_time_t(tp);

    // // 格式化为UTC时间字符串
    // std::tm* gmt = std::gmtime(&tt);
    // char buffer[30];
    // std::strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", gmt);

    // // 提取毫秒部分
    // std::chrono::milliseconds ms = std::chrono::duration_cast<std::chrono::milliseconds>(tp.time_since_epoch()) % 1000;
    
    // // 打印时间和毫秒部分
    // std::cout << "[" << buffer << "." << std::setw(3) << std::setfill('0') << ms.count() << " UTC] " << "odom_data: [x " << odom_data_pose_.pose.pose.position.x << ", y " << odom_data_pose_.pose.pose.position.y << ", yaw " << tf2::getYaw(odom_data_pose_.pose.pose.orientation) << "] "
    //     << "imu_data: [l_acc_x " << imu_data_.linear_acceleration.x << ", l_acc_y " << imu_data_.linear_acceleration.x << ", l_acc_z " << imu_data_.linear_acceleration.z << "]"
    //     << "[a_acc_x " << imu_data_.angular_velocity.x << ", a_acc_y " << imu_data_.angular_velocity.y << ", a_acc_z " << imu_data_.angular_velocity.z << "] imu_DataRecorder_yaw: " << tf2::getYaw(imu_data_.orientation) << ", "
    //     // << "imu: [l_acc_x " << imu_.linear_acceleration.x << ", l_acc_y " << imu_.linear_acceleration.x << ", l_acc_z " << imu_.linear_acceleration.z << "]"
    //     // << "[a_acc_x " << imu_.angular_velocity.x << ", a_acc_y " << imu_.angular_velocity.y << ", a_acc_z " << imu_.angular_velocity.z << "] imu_yaw: " << tf2::getYaw(imu_.orientation) << ", "
    //     // << "cmd_vel: [v " << vel_.linear.x << ", w " << vel_.angular.z << "]"
    //     << " estimate_pose: [x " << tf_pose_.x << ", y " << tf_pose_.y << ", yaw " << tf_pose_.z << "]" << std::endl;
}

// void DataRecorder::imu_sub_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
// {
//     this->imu_ = *msg;
// }

// void DataRecorder::imu_data_sub_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
// {
//     this->imu_data_ = *msg;
// }

void DataRecorder::vel_sub_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    this->vel_ = *msg;
}

}  // namespace data_recorder
