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

#ifndef ROBOT_FEATURE_DETECTION__FEATURE_DETECTION_HPP_
#define ROBOT_FEATURE_DETECTION__FEATURE_DETECTION_HPP_

#include <vector>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace feature_detection
{

struct smoothness_t {
    float value;
    size_t index;
};

struct by_value {
    bool operator()(smoothness_t const & left, smoothness_t const & right) {
        return left.value < right.value;
    }
};



class FeatureDetection : public rclcpp::Node
{
public:
    FeatureDetection();
    ~FeatureDetection();

private:
    void laserScanSubCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr featue_scan_pub_;

    float edge_threshold_; // 提取角点的阈值




};
}  // namespace feature_detection
#endif  // ROBOT_FEATURE_DETECTION__FEATURE_DETECTION_HPP_
