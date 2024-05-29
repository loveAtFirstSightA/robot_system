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

#include <iostream>
#include <cmath>
#include "nav2_amcl/icp/plicp.hpp"

namespace nav2_amcl
{
PLICP::PLICP()
{
    initParams();
    x_ = 0.0f;
    y_ = 0.0f;
    yaw_ = 0.0f;
}

PLICP::~PLICP()
{
    ld_free(scan_ldp_);
    ld_free(last_scan_ldp_);
}

void
PLICP::initParams()
{
    std::cout << getCurrentTime() << "Initialization plicp parameters" << std::endl;
    input_.max_angular_correction_deg = 45.0f;
    input_.max_linear_correction = 1.0f;
    input_.max_iterations = 100;
    input_.epsilon_xy = 0.000001f;
    input_.epsilon_theta = 0.000001f;
    input_.max_correspondence_dist = 1.0f;
    input_.sigma = 0.010f;
    input_.use_corr_tricks = 1;
    input_.restart = 0;
    input_.restart_threshold_mean_error = 0.01f;
    input_.restart_dt = 1.0f;
    input_.restart_dtheta = 0.1f;
    input_.clustering_threshold = 0.25f;
    input_.orientation_neighbourhood = 20;
    input_.use_point_to_line_distance = 1;
    input_.do_alpha_test = 0;
    input_.do_alpha_test_thresholdDeg = 20.0f;
    input_.outliers_maxPerc = 0.90f;
    input_.outliers_adaptive_order = 0.7f;
    input_.outliers_adaptive_mult = 2.0f;
    input_.do_visibility_test = 0;
    input_.outliers_remove_doubles = 1;
    input_.do_compute_covariance = 0;
    input_.debug_verify_tricks = 0;
    input_.use_ml_weights = 0;
    input_.use_sigma_weights = 0;
}

void
PLICP::createCache(const sensor_msgs::msg::LaserScan::SharedPtr & scan_msg)
{
    // 雷达数据间的角度是固定的，因此可以将对应角度的cos与sin值缓存下来，不用每次都计算
    a_cos_.clear();
    s_sin_.clear();
    for (size_t i = 0; i < scan_msg->ranges.size(); i++) {
        double angle = scan_msg->angle_min + scan_msg->angle_increment * i;
        a_cos_.push_back(std::cos(angle));
        s_sin_.push_back(std::sin(angle));
    }
    input_.min_reading = scan_msg->angle_min;
    input_.max_reading = scan_msg->angle_max;
}

#if 0
void
PLICP::convertMapToLDP(const nav_msgs::msg::OccupancyGrid::SharedPtr map)
{
    std::cout << getCurrentTime() << "Start converting map data" << std::endl;
    unsigned int width = static_cast<unsigned int>(map->info.width);
    unsigned int height = static_cast<unsigned int>(map->info.height);
    double resolution = static_cast<double>(map->info.resolution);
    double origin_x = map->info.origin.position.x;
    double origin_y = map->info.origin.position.y;
    std::cout << getCurrentTime() << "map: " << width << " * " << height << " " << resolution << " [" << origin_x << ", " << origin_y << "]" << std::endl;

    unsigned int sum_occ = 0;
    std::vector<ObstaclePoint> obstacle_points;

    for (size_t i = 0; i < map->data.size(); i++) {
        if (map->data[i] == 100) {
            sum_occ++;
            int row = i / width;
            int col = i % width;
            // Corrected the calculation of the actual position
            double x = origin_x + col * resolution;  // Column corresponds to x
            double y = origin_y + row * resolution;  // Row corresponds to y
            double dist = std::sqrt(x * x + y * y);
            double angle = std::atan2(y, x);

            obstacle_points.push_back({x, y, dist, angle});
        }
    }

    // Sort obstacle points by angle
    std::sort(obstacle_points.begin(), obstacle_points.end(), [](const ObstaclePoint &a, const ObstaclePoint &b) {
        return a.angle < b.angle;
    });

    // Convert sorted obstacle points to LDP
    map_ldp_ = ld_alloc_new(sum_occ);
    for (unsigned int i = 0; i < sum_occ; i++) {
        map_ldp_->valid[i] = 1;
        map_ldp_->readings[i] = obstacle_points[i].dist;
        map_ldp_->theta[i] = obstacle_points[i].angle;
        map_ldp_->cluster[i] = -1;
    }

    map_ldp_->min_theta = map_ldp_->theta[0];
    map_ldp_->max_theta = map_ldp_->theta[sum_occ - 1];
    // std::cout << getCurrentTime() << "map angle[" << map_ldp_->min_theta << ", " << map_ldp_->max_theta << "]" << std::endl;
    map_ldp_->odometry[0] = 0.0;
    map_ldp_->odometry[1] = 0.0;
    map_ldp_->odometry[2] = 0.0;
    map_ldp_->estimate[0] = 0.0;
    map_ldp_->estimate[1] = 0.0;
    map_ldp_->estimate[2] = 0.0;
    map_ldp_->true_pose[0] = 0.0;
    map_ldp_->true_pose[1] = 0.0;
    map_ldp_->true_pose[2] = 0.0;
}
#endif 

void
PLICP::convertScanToLDP(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
    unsigned int n = scan->ranges.size();
    scan_ldp_ = ld_alloc_new(n);
    for (unsigned int i = 0; i < n; i ++) {
        // calculate position in laser frame
        double r = scan->ranges[i];
        if (r > scan->range_min && r < scan->range_max) {
            // fill in laser scan data
            scan_ldp_->valid[i] = 1;
            scan_ldp_->readings[i] = r;
        } else {
            scan_ldp_->valid[i] = 0;
            scan_ldp_->readings[i] = -1;  //  for invalid range
        }
        scan_ldp_->theta[i] = scan->angle_min + scan->angle_increment * i;
        scan_ldp_->cluster[i] = -1;
    }
    scan_ldp_->min_theta = scan_ldp_->theta[0];
    scan_ldp_->max_theta = scan_ldp_->theta[n - 1];
    // std::cout << getCurrentTime() << "scan angle[" << scan_ldp_->min_theta << ", " << scan_ldp_->max_theta << "]" << std::endl;
    scan_ldp_->odometry[0] = 0.0;
    scan_ldp_->odometry[1] = 0.0;
    scan_ldp_->odometry[2] = 0.0;
    scan_ldp_->estimate[0] = 0.0;
    scan_ldp_->estimate[1] = 0.0;
    scan_ldp_->estimate[2] = 0.0;
    scan_ldp_->true_pose[0] = 0.0;
    scan_ldp_->true_pose[1] = 0.0;
    scan_ldp_->true_pose[2] = 0.0;

    if (!initialized_) {
        createCache(scan);
        last_scan_ldp_ = scan_ldp_;
        initialized_ = true;
        return;
    }
    processPLICP();
}

void
PLICP::processPLICP()
{
    input_.laser_ref = last_scan_ldp_;
    input_.laser_sens = scan_ldp_;
    input_.first_guess[0] = 0.0f;
    input_.first_guess[1] = 0.0f;
    input_.first_guess[2] = 0.0f;
    sm_icp(&input_, &output_);
    if (output_.valid) {
        // std::cout << getCurrentTime() << "plicp transfrom: (" << output_.x[0] << ", " << output_.x[1] << ", " << output_.x[2] << ")" << std::endl;
        x_ += output_.x[0];
        y_ += output_.x[1];
    } else {
        std::cout << getCurrentTime() << "Cannot perform conversion(plicp)" << std::endl;
    }
    ld_free(last_scan_ldp_);
    last_scan_ldp_ = scan_ldp_;
}

void
PLICP::getTransform(double & x, double & y)
{
    std::lock_guard<std::mutex> lock(mtx_);
    x = this->x_;
    y = this->y_;
}

}  // namespace nav2_amcl
