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

// 1 PL-ICP
// PL-ICP(Point to Line ICP) 使用点到线距离最小的方式进行ICP的计算，收敛速度快很多，同时精度也更高一些．

// 具体的pl-icp的介绍请看其论文，作者也开源了pl-icp的代码，作者将实现pl-icp的代码命名为csm( Canonical Scan Matcher).
// 论文和代码的详情在作者的官方网站 https://link.zhihu.com/?target=https%3A//censi.science/software/csm/

// 2 scan_tools
// ros中有人使用使用csm包进行了ros下的实现，进行了扫描匹配与位姿累加的实现，但是没有发布odometry的topic与tf．包的名字为 laser_scan_matcher，是scan_tools包集中的一个．

// 首先介绍一下scan_tools包集，这个包里提供了很多操作二维激光雷达数据的功能，虽然不一定能直接用，但是其处理数据的思路是非常值得借鉴的．
// 2.1 wiki地址
// wiki 链接 https://link.zhihu.com/?target=http%3A//wiki.ros.org/scan_tools%3Fdistro%3Dkinetic

// 2.2 功能简介
// 其包含的功能包名字如下所示，并对其功能进行了简要介绍．

// laser_ortho_projector: 将切斜的雷达数据投影到平面上．
// laser_scan_matcher: 基于pl-icp的扫描匹配的实现，并进行了位姿累加
// laser_scan_sparsifier: 对雷达数据进行稀疏处理
// laser_scan_splitter: 将一帧雷达数据分段，并发布出去
// ncd_parser: 读取New College Dataset，转换成ros的scan 与 odometry 发布出去
// polar_scan_matcher: 基于Polar Scan Matcher的扫描匹配器的ros实现
// scan_to_cloud_converter: 将 sensor_msgs/LaserScan 数据转成 sensor_msgs/PointCloud2 的数据格式．
// 我之前的那篇将雷达数据转成 sensor_msgs/PointCloud2 的文章，就是参考scan_to_cloud_converter包来实现的．

// 如果想要深入学习一下的请去wiki的网址，看介绍及源码，这里就不再过多介绍了．

#include "robot_pl_icp/pl_icp.hpp"

namespace pl_icp
{
PLICP::PLICP()
: Node("PL_ICP")
{
    laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        //   "/bcr_bot/scan",
        "scan",
        1,
        std::bind(&PLICP::laserScanSubCallback, this, std::placeholders::_1));
    
    initialized_ = false;
    initParams();
}

PLICP::~PLICP()
{
    // 
}

void
PLICP::laserScanSubCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    // 如果是第一帧数据，先进行初始化
    if (!initialized_) {
        // 将雷达各个角度的sin cos先保存起来，已节约计算量
        CreateCache(msg);
        // 转换数据格式为LDP
        convertScanToLDP(msg, pre_ldp_scan_);
        last_icp_time_ = msg->header.stamp;
        initialized_ = true;

        return;
    }
    // step 1 进行数据转换
    start_time_ = std::chrono::steady_clock::now();
    LDP current_lcp_scan;
    convertScanToLDP(msg, current_lcp_scan);
    end_time_ = std::chrono::steady_clock::now();
    used_time_ = std::chrono::duration_cast<std::chrono::duration<double>>(end_time_ - start_time_);
    // std::cout << "\n转换数据格式用时: " << used_time_.count() << " 秒。" << std::endl;

    // step 2 使用PLICP计算雷达前后两帧间的坐标变换
    start_time_ = std::chrono::steady_clock::now();
    scanMatchWithPLICP(current_lcp_scan, msg->header.stamp);
    end_time_ = std::chrono::steady_clock::now();
    used_time_ = std::chrono::duration_cast<std::chrono::duration<double>>(end_time_ - start_time_);
    if (used_time_.count() > 0.9f) {
        std::cout << "PLICP计算用时: " << used_time_.count() << " 秒。" << std::endl;
    }
}

void
PLICP::initParams()
{
    RCLCPP_INFO(this->get_logger(), "initialization params");
    this->declare_parameter<double>("max_angular_correction_deg", 45.0);
    this->declare_parameter<double>("max_linear_correction", 1.0);
    this->declare_parameter<int>("max_iterations", 100);
    this->declare_parameter<double>("epsilon_xy", 0.000001);
    this->declare_parameter<double>("epsilon_theta", 0.000001);
    this->declare_parameter<double>("max_correspondence_dist", 1.0);
    this->declare_parameter<double>("sigma", 0.010);
    this->declare_parameter<int>("use_corr_tricks", 1);
    this->declare_parameter<int>("restart", 0);
    this->declare_parameter<double>("restart_threshold_mean_error", 0.01);
    this->declare_parameter<double>("restart_dt", 1.0);
    this->declare_parameter<double>("restart_dtheta", 0.1);
    this->declare_parameter<double>("clustering_threshold", 0.25);
    this->declare_parameter<int>("orientation_neighbourhood", 20);
    this->declare_parameter<int>("use_point_to_line_distance", 1);
    this->declare_parameter<int>("do_alpha_test", 0);
    this->declare_parameter<double>("do_alpha_test_thresholdDeg", 20.0);
    this->declare_parameter<double>("outliers_maxPerc", 0.90);
    this->declare_parameter<double>("outliers_adaptive_order", 0.7);
    this->declare_parameter<double>("outliers_adaptive_mult", 2.0);
    this->declare_parameter<int>("do_visibility_test", 0);
    this->declare_parameter<int>("outliers_remove_doubles", 1);
    this->declare_parameter<int>("do_compute_covariance", 0);
    this->declare_parameter<int>("debug_verify_tricks", 0);
    this->declare_parameter<int>("use_ml_weights", 0);
    this->declare_parameter<int>("use_sigma_weights", 0);

    input_.max_angular_correction_deg = this->get_parameter("max_angular_correction_deg").as_double();
    input_.max_linear_correction = this->get_parameter("max_linear_correction").as_double();
    input_.max_iterations = this->get_parameter("max_iterations").as_int();
    input_.epsilon_xy = this->get_parameter("epsilon_xy").as_double();
    input_.epsilon_theta = this->get_parameter("epsilon_theta").as_double();
    input_.max_correspondence_dist = this->get_parameter("max_correspondence_dist").as_double();
    input_.sigma = this->get_parameter("sigma").as_double();
    input_.use_corr_tricks = this->get_parameter("use_corr_tricks").as_int();
    input_.restart = this->get_parameter("restart").as_int();
    input_.restart_threshold_mean_error = this->get_parameter("restart_threshold_mean_error").as_double();
    input_.restart_dt = this->get_parameter("restart_dt").as_double();
    input_.restart_dtheta = this->get_parameter("restart_dtheta").as_double();
    input_.clustering_threshold = this->get_parameter("clustering_threshold").as_double();
    input_.orientation_neighbourhood = this->get_parameter("orientation_neighbourhood").as_int();
    input_.use_point_to_line_distance = this->get_parameter("use_point_to_line_distance").as_int();
    input_.do_alpha_test = this->get_parameter("do_alpha_test").as_int();
    input_.do_alpha_test_thresholdDeg = this->get_parameter("do_alpha_test_thresholdDeg").as_double();
    input_.outliers_maxPerc = this->get_parameter("outliers_maxPerc").as_double();
    input_.outliers_adaptive_order = this->get_parameter("outliers_adaptive_order").as_double();
    input_.outliers_adaptive_mult = this->get_parameter("outliers_adaptive_mult").as_double();
    input_.do_visibility_test = this->get_parameter("do_visibility_test").as_int();
    input_.outliers_remove_doubles = this->get_parameter("outliers_remove_doubles").as_int();
    input_.do_compute_covariance = this->get_parameter("do_compute_covariance").as_int();
    input_.debug_verify_tricks = this->get_parameter("debug_verify_tricks").as_int();
    input_.use_ml_weights = this->get_parameter("use_ml_weights").as_int();
    input_.use_sigma_weights = this->get_parameter("use_sigma_weights").as_int();
}

void
PLICP::CreateCache(const sensor_msgs::msg::LaserScan::SharedPtr & scan_msg)
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

void
PLICP::convertScanToLDP(const sensor_msgs::msg::LaserScan::SharedPtr & scan_msg, LDP & ldp)
{
    unsigned int n = scan_msg->ranges.size();
    ldp = ld_alloc_new(n);
    for (unsigned int i = 0; i < n; i ++) {
        // calculate position in laser frame
        double r = scan_msg->ranges[i];
        if (r > scan_msg->range_min && r < scan_msg->range_max) {
            // fill in laser scan data
            ldp->valid[i] = 1;
            ldp->readings[i] = r;
        } else {
            ldp->valid[i] = 0;
            ldp->readings[i] = -1;  //  for invalid range
        }
        ldp->theta[i] = scan_msg->angle_min + scan_msg->angle_increment * i;
        ldp->cluster[i] = -1;
    }
    ldp->min_theta = ldp->theta[0];
    ldp->max_theta = ldp->theta[n - 1];
    
    ldp->odometry[0] = 0.0;
    ldp->odometry[1] = 0.0;
    ldp->odometry[2] = 0.0;
    ldp->estimate[0] = 0.0;
    ldp->estimate[1] = 0.0;
    ldp->estimate[2] = 0.0;
    ldp->true_pose[0] = 0.0;
    ldp->true_pose[1] = 0.0;
    ldp->true_pose[2] = 0.0;
}

void
PLICP::scanMatchWithPLICP(LDP & curr_ldp_scan, const rclcpp::Time & time)
{
    // CSM is used in the following way:
    // The scans are always in the laser frame
    // The reference scan (prevLDPcan_) has a pose of [0, 0, 0]
    // The new scan (currLDPScan) has a pose equal to the movement
    // of the laser in the laser frame since the last scan
    // The computed correction is then propagated using the tf machinery
    input_.laser_ref = pre_ldp_scan_;
    input_.laser_sens = curr_ldp_scan;

    // 位置的预测数值为0 就是不能预测
    input_.first_guess[0] = 0.0f;
    input_.first_guess[1] = 0.0f;
    input_.first_guess[2] = 0.0f;

    // 调用csm里的函数进行plicp计算帧间的匹配，输出结果保存在output里
    sm_icp(&input_, &output_);
    if (output_.valid) {
        // std::cout << "transfrom: (" << output_.x[0] << ", " << output_.x[1] << ", " 
        //     << output_.x[2] * 180 / M_PI << ")" << std::endl;
        std::cout << "transfrom: (" << output_.x[0] << ", " << output_.x[1] << ", " 
            << output_.x[2] << ")" << std::endl;
    } else {
        std::cout << "not Converged" << std::endl;
    }
    // 删除prev_ldp_scan_，用curr_ldp_scan进行替代
    ld_free(pre_ldp_scan_);
    pre_ldp_scan_ = curr_ldp_scan;
    last_icp_time_ = time;
}



}  // namespace pl_icp
