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
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace data_recorder
{
DataRecorder::DataRecorder() : Node("data_recorder")
{
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(15),
        std::bind(&DataRecorder::timer_callback, this));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom",
        100,
        std::bind(&DataRecorder::odom_sub_callback, this, std::placeholders::_1));
    odom_data_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom_data",
        100,
        std::bind(&DataRecorder::odom_data_sub_callback, this, std::placeholders::_1));
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu",
        100,
        std::bind(&DataRecorder::imu_sub_callback, this, std::placeholders::_1));
    imu_data_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu_data",
        100,
        std::bind(&DataRecorder::imu_data_sub_callback, this, std::placeholders::_1));
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
    std::string source_link = "base_footprint";
    std::string target_link = "map";
    geometry_msgs::msg::TransformStamped tf;
    try {
        tf = tf_buffer_->lookupTransform(target_link, source_link, tf2::TimePointZero);
        tf_pose_.x = tf.transform.translation.x;
        tf_pose_.y = tf.transform.translation.y;
        tf_pose_.z = tf2::getYaw(tf.transform.rotation);
        localization_ = true;
    } catch(const std::exception& e) {
        // spdlog::warn("Not TF transform");
        localization_ = false;
    }
}

void DataRecorder::odom_sub_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    if (!localization_) {
        return;
    }
    this->odom_pose_ = *msg;
    // spdlog::info("odom: [x {:.4f}, y {:.4f}, yaw {:.4f}] vel: [v {:.4f}, w {:.4f}] estimate: [x {:.4f}, y {:.4f}, yaw {:.4f}] encoder: [x {:.4f}, y {:.4f}, yaw {:.4f}] imu: linear_acc [x {:.4f}, y {:.4f}, z {:.4f}] angular_acc [x {:.4f}, y {:.4f}, z {:.4f}] yaw [{:.4f}] imu_data: linear_acc [x {:.4f}, y {:.4f}, z {:.4f}] angular_acc [x {:.4f}, y {:.4f}, z {:.4f}] yaw [{:.4f}]",
    //     odom_pose_.pose.pose.position.x, odom_pose_.pose.pose.position.y, tf2::getYaw(odom_pose_.pose.pose.orientation),
    //     vel_.linear.x, vel_.angular.z,
    //     tf_pose_.x, tf_pose_.y, tf_pose_.z,
    //     odom_data_pose_.pose.pose.position.x, odom_data_pose_.pose.pose.position.y, tf2::getYaw(odom_data_pose_.pose.pose.orientation),
    //     imu_.linear_acceleration.x, imu_.linear_acceleration.y, imu_.linear_acceleration.z,
    //     imu_.angular_velocity.x, imu_.angular_velocity.y, imu_.angular_velocity.z,
    //     tf2::getYaw(imu_.orientation),
    //     imu_data_.linear_acceleration.x, imu_data_.linear_acceleration.y, imu_data_.linear_acceleration.z,
    //     imu_data_.angular_velocity.x, imu_data_.angular_velocity.y, imu_data_.angular_velocity.z,
    //     tf2::getYaw(imu_data_.orientation));
    // simple
    spdlog::info("odom: [x {:.4f}, y {:.4f}, yaw {:.4f}] vel: [v {:.4f}, w {:.4f}] estimate: [x {:.4f}, y {:.4f}, yaw {:.4f}] encoder: [x {:.4f}, y {:.4f}, yaw {:.4f}]",
        odom_pose_.pose.pose.position.x, odom_pose_.pose.pose.position.y, tf2::getYaw(odom_pose_.pose.pose.orientation),
        vel_.linear.x, vel_.angular.z,
        tf_pose_.x, tf_pose_.y, tf_pose_.z,
        odom_data_pose_.pose.pose.position.x, odom_data_pose_.pose.pose.position.y, tf2::getYaw(odom_data_pose_.pose.pose.orientation));
}

void DataRecorder::odom_data_sub_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    if (!localization_) {
        return;
    }
    this->odom_data_pose_ = *msg;
}

void DataRecorder::imu_sub_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    if (!localization_) {
        return;
    }
    this->imu_ = *msg;
}

void DataRecorder::imu_data_sub_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    if (!localization_) {
        return;
    }
    this->imu_data_ = *msg;
}

void DataRecorder::vel_sub_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    if (!localization_) {
        return;
    }
    this->vel_ = *msg;
}

}  // namespace data_recorder
