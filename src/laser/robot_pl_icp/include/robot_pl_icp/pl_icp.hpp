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

#ifndef ROBOT_PL_ICP__PL_ICP_HPP_
#define ROBOT_PL_ICP__PL_ICP_HPP_

#include <chrono>
#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "csm/csm.h"

namespace pl_icp
{
class PLICP : public rclcpp::Node
{
public:
    PLICP();
    ~PLICP();

private:
    void laserScanSubCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    
    void initParams();
    void CreateCache(const sensor_msgs::msg::LaserScan::SharedPtr & scan_msg);
    void convertScanToLDP(const sensor_msgs::msg::LaserScan::SharedPtr & scan_msg, LDP & ldp);
    void scanMatchWithPLICP(LDP & curr_ldp_scan, const rclcpp::Time & time);

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;

    std::chrono::steady_clock::time_point start_time_;
    std::chrono::steady_clock::time_point end_time_;
    std::chrono::duration<double> used_time_;

    bool initialized_{false};

    std::vector<double> a_cos_;
    std::vector<double> s_sin_;

    // csm
    sm_params input_;
    sm_result output_;
    LDP pre_ldp_scan_;  //  LDP - aser data points

    rclcpp::Time last_icp_time_;




};
}  // namespace pl_icp
#endif  // ROBOT_PL_ICP__PL_ICP_HPP_
