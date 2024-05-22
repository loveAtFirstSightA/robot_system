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

#ifndef ROBOT_ITERATIVE_CLOSEST_POINT__ITERATIVE_CLOSEST_POINT_HPP_
#define ROBOT_ITERATIVE_CLOSEST_POINT__ITERATIVE_CLOSEST_POINT_HPP_

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

// pcl header file
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/registration/icp.h"

namespace iterative_closest_point
{
class IterativeClosestPoint : public rclcpp::Node
{
public:
    IterativeClosestPoint();
    ~IterativeClosestPoint();

private:
    void laserScanSubCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    // 点云数据格式转换成PCL格式
    void converScanToPcl(const sensor_msgs::msg::LaserScan::SharedPtr & scan);
    // 扫描匹配
    void scanMatchWithICP();

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;

    // 第一帧雷达数据到来的标志，因为第一帧数据到来时，只有一帧数据，是没办法进行匹配的，所以要对第一帧数据进行特殊处理
    bool is_first_scan_{true};
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_pointcloud_;    //  当前帧雷达数据
    pcl::PointCloud<pcl::PointXYZ>::Ptr last_pointcloud_;   //  上一帧雷达数据
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp_;





};
}  // namespace iterative_closest_point
#endif  // ROBOT_ITERATIVE_CLOSEST_POINT__ITERATIVE_CLOSEST_POINT_HPP_
