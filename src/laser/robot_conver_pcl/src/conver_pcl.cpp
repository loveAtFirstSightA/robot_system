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

#include "robot_conver_pcl/conver_pcl.hpp"

namespace conver_pcl
{

ConverPcl::ConverPcl()
: Node ("robot_conver_pcl")
{
     laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
          "/bcr_bot/scan",
          1,
          std::bind(&ConverPcl::laserScanSubCallback, this, std::placeholders::_1));
     pcl_scan_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
          "pointcloud2_converted",
          1);
     // Preallocate the point cloud
     cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
}

ConverPcl::~ConverPcl()
{

}

void
ConverPcl::laserScanSubCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
     cloud_->clear();
     cloud_->reserve(msg->ranges.size());  // Reserve memory to avoid multiple allocations

     const float angle_min = msg->angle_min;
     const float angle_increment = msg->angle_increment;

     for (size_t i = 0; i < msg->ranges.size(); ++i) {
          float range = msg->ranges[i];
          if (std::isfinite(range)) {
               float angle = angle_min + i * angle_increment;
               pcl::PointXYZ point;
               point.x = range * std::cos(angle);
               point.y = range * std::sin(angle);
               point.z = 0.0; // Assuming planar laser scan
               cloud_->push_back(point);
          }
     }

     sensor_msgs::msg::PointCloud2 pcl_msg;
     pcl::toROSMsg(*cloud_, pcl_msg);
     pcl_msg.header = msg->header;
     pcl_scan_pub_->publish(pcl_msg);
}


}  // namespace conver_pcl
