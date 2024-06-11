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

#ifndef ITERATIVE_CLOSEST_POINT__SCAN_TO_MAP_ICP_HPP_
#define ITERATIVE_CLOSEST_POINT__SCAN_TO_MAP_ICP_HPP_

#include "rclcpp/rclcpp.hpp"
#include "iterative_closest_point/iterative_closest_point.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "iterative_closest_point/scan_to_map_icp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/utils.h"

namespace iterative_closest_point
{
class ScanToMapICP : public rclcpp::Node
{
public:
    ScanToMapICP();
    ~ScanToMapICP();

private:
    void mapSubCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void scanSubCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    bool calculateTfLidarToMap(geometry_msgs::msg::TransformStamped & tf);
    
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

    iterative_closest_point::ICP icp_;


};
}  // namespace iterative_closest_point
#endif  // ITERATIVE_CLOSEST_POINT__SCAN_TO_MAP_ICP__SCAN_TO_MAP_ICP_HPP_
