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

#ifndef PARTICLE_FILTER__PARTICLE_FILTER_HPP_
#define PARTICLE_FILTER__PARTICLE_FILTER_HPP_

#include <math.h>
#include "rclcpp/rclcpp.hpp"
#include "particle_filter/logger.hpp"
// #include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"

namespace particle_filter
{
struct pf_pose {
    double x;
    double y;
    double yaw;
};

struct pf {
    pf_pose pose;
    double weight;
};

class ParticleFilter : public rclcpp::Node
{
public:
    ParticleFilter();
    ~ParticleFilter();

private:
    void initParticleFilter();
    // 读取激光雷达数据 里程计数据 地图数据
    void mapSubCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void scanSubCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    bool getOdom(double & x, double & y, double & yaw);

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    bool map_received_{false};
    nav_msgs::msg::OccupancyGrid map_;

private:
    double normalize(double angle);
    double angleDiff(double a, double b);
    void sampleMotionModelOdometry();



};
}  // namespace particle_filter
#endif  // PARTICLE_FILTER__PARTICLE_FILTER_HPP_
