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

#include "iterative_closest_point/scan_to_map_icp.hpp"

namespace iterative_closest_point
{
ScanToMapICP::ScanToMapICP()
: Node("scan_to_map_icp")
{
     map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
          "map",
          10,
          std::bind(&ScanToMapICP::scanSubCallback, this, std::placeholders::_1)); 
     scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
          "scan",
          10,
          std::bind(&ScanToMapICP::scanSubCallback, this, std::placeholders::_1));   
}

ScanToMapICP::~ScanToMapICP()
{

}


void
ScanToMapICP::mapSubCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
     RCLCPP_INFO(this->get_logger(), "Received message");
}
void
ScanToMapICP::scanSubCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
     RCLCPP_INFO(this->get_logger(), "Received message");
}


}  // namespace iterative_closest_point
