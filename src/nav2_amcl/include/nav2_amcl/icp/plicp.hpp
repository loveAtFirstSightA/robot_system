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

#ifndef NAV2_AMCL__ICP__PLICP_HPP_
#define NAV2_AMCL__ICP__PLICP_HPP_

#include <vector>
#include <chrono>
#include <memory>
#include <mutex>
#include "nav2_amcl/logger.hpp"
#include "csm/csm.h"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace nav2_amcl
{
#if 0
// Define a struct to hold obstacle point data with coordinates and angle
struct ObstaclePoint {
    double x;
    double y;
    double dist;
    double angle;
};
#endif
class PLICP
{
public:
    PLICP();
    ~PLICP();
    void convertScanToLDP(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    void getTransform(double & x, double & y);

private:
    void initParams();
    void createCache(const sensor_msgs::msg::LaserScan::SharedPtr & scan_msg);
    void processPLICP();

    bool initialized_{false};
    sm_params input_;
    sm_result output_;
    std::vector<double> a_cos_;
    std::vector<double> s_sin_;
    LDP scan_ldp_;
    LDP last_scan_ldp_;

    double x_;
    double y_;
    double yaw_;
    std::mutex mtx_;
};
}  // namespace nav2_amcl
#endif  // NAV2_AMCL__ICP__PLICP_HPP_
