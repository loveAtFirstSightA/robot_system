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

#ifndef ROBOT_PLICP_ODOM__PLICP_ODOM_HPP_
#define ROBOT_PLICP_ODOM__PLICP_ODOM_HPP_

#include <memory>
#include <vector>
#include <chrono>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "csm/csm.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/utils.h"  //  数据格式转换
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace plicp_odom
{
class PLICPOdom : public rclcpp::Node
{
public:
    PLICPOdom();
    ~PLICPOdom();

private:
    void laserScanSubCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    
    void initParams();
    void createCache(const sensor_msgs::msg::LaserScan::SharedPtr & scan_msg);
    void convertScanToLDP(const sensor_msgs::msg::LaserScan::SharedPtr & scan_msg, LDP & ldp);
    void scanMatchWithPLICP(LDP & curr_ldp_scan, const rclcpp::Time & time);

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

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

    // Transform相关 声明
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf2_broadcaster_;
    std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;

    std::string odom_frame_;
    std::string base_frame_;

    double kf_dist_linear_;
    double kf_dist_linear_sq_;
    double kf_dist_angular_;

    // ? no params ?
    int kf_scan_count_;
    int scan_count_;

    bool getBaseToLaserTf(const std::string & frame_id);

    tf2::Transform base_to_laser_;    
    tf2::Transform laser_to_base_; 

    tf2::Transform base_in_odom_;           // base_link在odom坐标系下的坐标
    tf2::Transform base_in_odom_keyframe_;  // base_link在odom坐标系下的keyframe的坐标

    void getPrediction(double & prediction_change_x, double & prediction_change_y, double & prediction_change_angle, double dt);

    geometry_msgs::msg::Twist last_velocity_;

    void createTfFromXYTheta(double x, double y, double theta, tf2::Transform& t);

    bool NewKeyframeNeeded(const tf2::Transform &d);



};
}  // namespace plicp_odom
#endif  // ROBOT_PLICP_ODOM__PLICP_ODOM_HPP_
