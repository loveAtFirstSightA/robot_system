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

#ifndef ROBOT_CONVER_PCL__CONVER_PCL_HPP_
#define ROBOT_CONVER_PCL__CONVER_PCL_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

// pcl
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

namespace conver_pcl
{
class ConverPcl : public rclcpp::Node
{
    // // 使用PCL中点的数据结构 pcl::PointXYZ
    // typedef pcl::PointXYZ PointT;
    // // 使用PCL中点云的数据结构 pcl::PointCloud<pcl::PointXYZ>
    // typedef pcl::PointCloud<PointT> PointCloudT;
public:
    ConverPcl();
    ~ConverPcl();

private:
    void laserScanSubCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_scan_pub_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;



};
}  // namespace conver_pcl
#endif  // ROBOT_CONVER_PCL__CONVER_PCL_HPP_
