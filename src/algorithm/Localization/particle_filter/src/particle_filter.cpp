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

#include "particle_filter/particle_filter.hpp"

namespace particle_filter
{
ParticleFilter::ParticleFilter() : Node("particle_filter")
{
     tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
     tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
     tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
     map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
          "/map",
          rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
          std::bind(&ParticleFilter::mapSubCallback, this, std::placeholders::_1));
     scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
          "/scan",
          rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data)),
          std::bind(&ParticleFilter::scanSubCallback, this, std::placeholders::_1));
}

ParticleFilter::~ParticleFilter() {}

void ParticleFilter::mapSubCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
     map_ = *msg;
     map_received_ = true;
}

void ParticleFilter::scanSubCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
     if (!map_received_) {
          std::cout << getCurrentTime() << "No map reveiced" << std::endl;
          return;
     }
     Pose odom;
     // 获取里程计信息
     if (!getOdom(odom.x, odom.y, odom.yaw)) {
          std::cout << getCurrentTime() << "No odometry reveiced" << std::endl;
          return;
     }

}

bool ParticleFilter::getOdom(double & x, double & y, double & yaw)
{
     geometry_msgs::msg::TransformStamped tf_pose;
     try {
          tf_pose = tf_buffer_->lookupTransform("base_footprint", "odom", tf2::TimePointZero);
     } catch(const std::exception& e) {
          std::cerr << getCurrentTime() << e.what() << '\n';
          return false;
     }
     x = tf_pose.transform.translation.x;
     y = tf_pose.transform.translation.y;
     yaw = tf2::getYaw(tf_pose.transform.rotation);
     return true;
}


}  // namespace particle_filter
