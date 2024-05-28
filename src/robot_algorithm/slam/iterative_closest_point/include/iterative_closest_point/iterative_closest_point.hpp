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

#ifndef ITERATIVE_CLOSEST_POINT__ITERATIVE_CLOSEST_POINT_HPP_
#define ITERATIVE_CLOSEST_POINT__ITERATIVE_CLOSEST_POINT_HPP_

// logger
#include <iostream>
#include <iomanip>
#include <ctime>
#include <sstream>
#include <string>

// pcl header file
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/registration/icp.h"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"

namespace iterative_closest_point
{
class ICP
{
public:
    ICP();
    ~ICP();
    void processICP();

    // 转换地图格式为PCL
    void converMapToPcl(const nav_msgs::msg::OccupancyGrid::SharedPtr map);

    // 转换scan格式为PCL
    void converScanToPcl(const sensor_msgs::msg::LaserScan::SharedPtr scan, double x, double y, double yaw);

private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_pointcloud_;    //  当前激光点集合
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_pointcloud_;   //  栅格地图点集
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp_;

};
}  // namespace iterative_closest_point
#endif  // ITERATIVE_CLOSEST_POINT__ITERATIVE_CLOSEST_POINT_HPP_
