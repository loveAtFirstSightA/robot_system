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

#include "robot_plicp_odom/plicp_odom.hpp"

namespace plicp_odom
{
PLICPOdom::PLICPOdom()
: Node("pl_icp")
{
    laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        //   "/bcr_bot/scan",
        "scan",
          1,
          std::bind(&PLICPOdom::laserScanSubCallback, this, std::placeholders::_1));
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
        "/plicp_odom",
        1);
    
    initialized_ = false;
    initParams();

    // 初始化TF
    tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
    tf2_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
}

PLICPOdom::~PLICPOdom()
{
    // 
}

// 主函数
void
PLICPOdom::laserScanSubCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    if (!initialized_) {
        // 将每个激光点的角度进行缓存
        createCache(msg);
        // 获取机器人的坐标系 base to laser
        if (!getBaseToLaserTf(msg->header.frame_id)) {
            RCLCPP_WARN(this->get_logger(), "Skipping scan");
            return;
        }
        convertScanToLDP(msg, pre_ldp_scan_);
        last_icp_time_ = msg->header.stamp;
        initialized_ = true;
        return;
    }
    // step 1 进行数据转换
    start_time_ = std::chrono::steady_clock::now();
    LDP current_ldp_scan;
    convertScanToLDP(msg, current_ldp_scan);
    end_time_ = std::chrono::steady_clock::now();
    used_time_ = std::chrono::duration_cast<std::chrono::duration<double>>(end_time_ - start_time_);
    std::cout << "\n转换数据格式用时: " << used_time_.count() << " 秒。" << std::endl;

    // step 2 使用PLICP计算雷达前后两帧间的坐标变换
    start_time_ = std::chrono::steady_clock::now();
    scanMatchWithPLICP(current_ldp_scan, msg->header.stamp);
    used_time_ = std::chrono::duration_cast<std::chrono::duration<double>>(end_time_ - start_time_);
    std::cout << "\n整体函数处理用时: " << used_time_.count() << " 秒。" << std::endl;
    
}


// 初始化参数
void
PLICPOdom::initParams()
{
    RCLCPP_INFO(this->get_logger(), "Initiate parameters");

    // Declare parameters with default values
    this->declare_parameter<std::string>("odom_frame", "odom");
    // this->declare_parameter<std::string>("base_frame", "base_link");
    this->declare_parameter<std::string>("base_frame", "base_footprint");

    // **** keyframe params: when to generate the keyframe scan
    // if either is set to 0, reduces to frame-to-frame matching
    this->declare_parameter<double>("kf_dist_linear", 0.1);
    this->declare_parameter<double>("kf_dist_angular", 5.0 * (M_PI / 180.0));

    // **** CSM 的参数 - comments copied from algos.h (by Andrea Censi)

    // Maximum angular displacement between scans
    this->declare_parameter<double>("max_angular_correction_deg", 45.0);

    // Maximum translation between scans (m)
    this->declare_parameter<double>("max_linear_correction", 0.50);

    // Maximum ICP cycle iterations
    this->declare_parameter<int>("max_iterations", 10);

    // A threshold for stopping (m)
    this->declare_parameter<double>("epsilon_xy", 0.000001);

    // A threshold for stopping (rad)
    this->declare_parameter<double>("epsilon_theta", 0.000001);

    // Maximum distance for a correspondence to be valid
    this->declare_parameter<double>("max_correspondence_dist", 0.3);

    // Noise in the scan (m)
    this->declare_parameter<double>("sigma", 0.010);

    // Use smart tricks for finding correspondences.
    this->declare_parameter<int>("use_corr_tricks", 1);

    // Restart: Restart if error is over threshold
    this->declare_parameter<int>("restart", 0);

    // Restart: Threshold for restarting
    this->declare_parameter<double>("restart_threshold_mean_error", 0.01);

    // Restart: displacement for restarting. (m)
    this->declare_parameter<double>("restart_dt", 1.0);

    // Restart: displacement for restarting. (rad)
    this->declare_parameter<double>("restart_dtheta", 0.1);

    // Max distance for staying in the same clustering
    this->declare_parameter<double>("clustering_threshold", 0.25);

    // Number of neighbour rays used to estimate the orientation
    this->declare_parameter<int>("orientation_neighbourhood", 20);

    // If 0, it's vanilla ICP
    this->declare_parameter<int>("use_point_to_line_distance", 1);

    // Discard correspondences based on the angles
    this->declare_parameter<int>("do_alpha_test", 0);

    // Discard correspondences based on the angles - threshold angle, in degrees
    this->declare_parameter<double>("do_alpha_test_thresholdDeg", 20.0);

    // Percentage of correspondences to consider: if 0.9,
    // always discard the top 10% of correspondences with more error
    this->declare_parameter<double>("outliers_maxPerc", 0.90);

    // Parameters describing a simple adaptive algorithm for discarding.
    //  1) Order the errors.
    //  2) Choose the percentile according to outliers_adaptive_order.
    //     (if it is 0.7, get the 70% percentile)
    //  3) Define an adaptive threshold multiplying outliers_adaptive_mult
    //     with the value of the error at the chosen percentile.
    //  4) Discard correspondences over the threshold.
    //  This is useful to be conservative; yet remove the biggest errors.
    this->declare_parameter<double>("outliers_adaptive_order", 0.7);
    this->declare_parameter<double>("outliers_adaptive_mult", 2.0);

    // If you already have a guess of the solution, you can compute the polar angle
    // of the points of one scan in the new position. If the polar angle is not a monotone
    // function of the readings index, it means that the surface is not visible in the
    // next position. If it is not visible, then we don't use it for matching.
    this->declare_parameter<int>("do_visibility_test", 0);

    // no two points in laser_sens can have the same corr.
    this->declare_parameter<int>("outliers_remove_doubles", 1);

    // If 1, computes the covariance of ICP using the method http://purl.org/censi/2006/icpcov
    this->declare_parameter<int>("do_compute_covariance", 0);

    // Checks that find_correspondences_tricks gives the right answer
    this->declare_parameter<int>("debug_verify_tricks", 0);

    // If 1, the field 'true_alpha' (or 'alpha') in the first scan is used to compute the
    // incidence beta, and the factor (1/cos^2(beta)) used to weight the correspondence.
    this->declare_parameter<int>("use_ml_weights", 0);

    // If 1, the field 'readings_sigma' in the second scan is used to weight the
    // correspondence by 1/sigma^2
    this->declare_parameter<int>("use_sigma_weights", 0);

    // Get parameter values
    this->get_parameter("odom_frame", odom_frame_);
    this->get_parameter("base_frame", base_frame_);
    this->get_parameter("kf_dist_linear", kf_dist_linear_);
    this->get_parameter("kf_dist_angular", kf_dist_angular_);
    kf_dist_linear_sq_ = kf_dist_linear_ * kf_dist_linear_;

    this->get_parameter("max_angular_correction_deg", input_.max_angular_correction_deg);
    this->get_parameter("max_linear_correction", input_.max_linear_correction);
    this->get_parameter("max_iterations", input_.max_iterations);
    this->get_parameter("epsilon_xy", input_.epsilon_xy);
    this->get_parameter("epsilon_theta", input_.epsilon_theta);
    this->get_parameter("max_correspondence_dist", input_.max_correspondence_dist);
    this->get_parameter("sigma", input_.sigma);
    this->get_parameter("use_corr_tricks", input_.use_corr_tricks);
    this->get_parameter("restart", input_.restart);
    this->get_parameter("restart_threshold_mean_error", input_.restart_threshold_mean_error);
    this->get_parameter("restart_dt", input_.restart_dt);
    this->get_parameter("restart_dtheta", input_.restart_dtheta);
    this->get_parameter("clustering_threshold", input_.clustering_threshold);
    this->get_parameter("orientation_neighbourhood", input_.orientation_neighbourhood);
    this->get_parameter("use_point_to_line_distance", input_.use_point_to_line_distance);
    this->get_parameter("do_alpha_test", input_.do_alpha_test);
    this->get_parameter("do_alpha_test_thresholdDeg", input_.do_alpha_test_thresholdDeg);
    this->get_parameter("outliers_maxPerc", input_.outliers_maxPerc);
    this->get_parameter("outliers_adaptive_order", input_.outliers_adaptive_order);
    this->get_parameter("outliers_adaptive_mult", input_.outliers_adaptive_mult);
    this->get_parameter("do_visibility_test", input_.do_visibility_test);
    this->get_parameter("outliers_remove_doubles", input_.outliers_remove_doubles);
    this->get_parameter("do_compute_covariance", input_.do_compute_covariance);
    this->get_parameter("debug_verify_tricks", input_.debug_verify_tricks);
    this->get_parameter("use_ml_weights", input_.use_ml_weights);
    this->get_parameter("use_sigma_weights", input_.use_sigma_weights);
}

void
PLICPOdom::createCache(const sensor_msgs::msg::LaserScan::SharedPtr & scan_msg)
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
PLICPOdom::convertScanToLDP(const sensor_msgs::msg::LaserScan::SharedPtr & scan_msg, LDP & ldp)
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
    // ldp->estimate[0] = 0.0;
    // ldp->estimate[1] = 0.0;
    // ldp->estimate[2] = 0.0;
    ldp->true_pose[0] = 0.0;
    ldp->true_pose[1] = 0.0;
    ldp->true_pose[2] = 0.0;
}

void
PLICPOdom::scanMatchWithPLICP(LDP & curr_ldp_scan, const rclcpp::Time & time)
{
    // CSM is used in the following way:
    // The scans are always in the laser frame
    // The reference scan (prevLDPcan_) has a pose of [0, 0, 0]
    // The new scan (currLDPScan) has a pose equal to the movement
    // of the laser in the laser frame since the last scan
    // The computed correction is then propagated using the tf machinery

    pre_ldp_scan_->odometry[0] = 0.0f;
    pre_ldp_scan_->odometry[1] = 0.0f;
    pre_ldp_scan_->odometry[2] = 0.0f;

    pre_ldp_scan_->estimate[0] = 0.0f;
    pre_ldp_scan_->estimate[1] = 0.0f;
    pre_ldp_scan_->estimate[2] = 0.0f;

    pre_ldp_scan_->true_pose[0] = 0.0f;
    pre_ldp_scan_->true_pose[1] = 0.0f;
    pre_ldp_scan_->true_pose[2] = 0.0f;



    input_.laser_ref = pre_ldp_scan_;
    input_.laser_sens = curr_ldp_scan;

    // 使用匀速模型 速度 * 时间 得到预测的odom坐标下的位姿变化
    double dt = (now() - last_icp_time_).nanoseconds() / 1e+9;
    double pre_ch_x, pre_ch_y, pre_ch_a;
    getPrediction(pre_ch_x, pre_ch_y, pre_ch_a, dt);

    tf2::Transform prediction_change;
    createTfFromXYTheta(pre_ch_x, pre_ch_y, pre_ch_a, prediction_change);

    // 将odom坐标系下的坐标变换转换成 base_in_odom_keyframe_坐标系下的坐标变换
    prediction_change = prediction_change * (base_in_odom_ * base_in_odom_keyframe_.inverse());

    // 将baselink坐标下的坐标变换 转换成 雷达坐标系下的坐标变换
    tf2::Transform prediction_change_lidar;
    prediction_change_lidar = laser_to_base_ * base_in_odom_.inverse() * prediction_change * base_in_odom_ * base_to_laser_;
    
    input_.first_guess[0] = prediction_change_lidar.getOrigin().getX();
    input_.first_guess[1] = prediction_change_lidar.getOrigin().getY();
    input_.first_guess[2] = tf2::getYaw(prediction_change_lidar.getRotation());

    // // 位置的预测数值为0 就是不能预测
    // input_.first_guess[0] = 0.0f;
    // input_.first_guess[1] = 0.0f;
    // input_.first_guess[2] = 0.0f;

    // If they are non-Null, free covariance gsl matrices to avoid leaking memory
    if (output_.cov_x_m) {
        gsl_matrix_free(output_.cov_x_m);
        output_.cov_x_m = 0;
    }
    if (output_.dx_dy1_m) {
        gsl_matrix_free(output_.dx_dy1_m);
        output_.dx_dy1_m = 0;
    }
    if (output_.dx_dy2_m) {
        gsl_matrix_free(output_.dx_dy2_m);
        output_.dx_dy2_m = 0;
    }

    // start_time_ = std::chrono::steady_clock::now();
    // 调用csm里的函数进行plicp计算帧间的匹配，输出结果保存在output里
    sm_icp(&input_, &output_);
    // end_time_ = std::chrono::steady_clock::now();
    // used_time_ = std::chrono::duration_cast<std::chrono::duration<double>>(end_time_ - start_time_);
    // std::cout << "PLICP计算用时: " << time_used_.count() << " 秒。" << std::endl;
    tf2::Transform correct_ch;

    if (output_.valid) {
        // 雷达坐标系下的坐标变换
        tf2::Transform correct_ch_l;
        createTfFromXYTheta(output_.x[0], output_.x[1], output_.x[2], correct_ch_l);
        // 将雷达坐标系下的坐标变换 转换成 base_link坐标下的坐标变换
        correct_ch = base_to_laser_ * correct_ch_l * laser_to_base_;
        // 更新baselink在odom坐标系下的坐标
        base_in_odom_ = base_in_odom_keyframe_ * correct_ch;
        last_velocity_.linear.x = correct_ch.getOrigin().getX() / dt;
        last_velocity_.angular.z = tf2::getYaw(correct_ch.getRotation()) / dt;

        // std::cout << "transfrom: (" << output_.x[0] << ", " << output_.x[1] << ", " 
            // << output_.x[2] * 180 / M_PI << ")" << std::endl;
    } else {
        std::cout << "not Converged" << std::endl;
    }
    // publisher odometry
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = now();
    tf_msg.header.frame_id = odom_frame_;
    tf_msg.child_frame_id = base_frame_;
    tf_msg.transform = tf2::toMsg(base_in_odom_);

    // 发布 odom 到 base_link 的 tf
    tf2_broadcaster_->sendTransform(tf_msg);

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = now();
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = base_frame_;
    tf2::toMsg(base_in_odom_, odom_msg.pose.pose);
    odom_msg.twist.twist = last_velocity_;

    // 发布 odomemtry 话题
    odom_pub_->publish(odom_msg);

    // 检查是否更新关键帧坐标
    if (NewKeyframeNeeded(correct_ch)) {
        ld_free(pre_ldp_scan_);
        pre_ldp_scan_ = curr_ldp_scan;
        base_in_odom_keyframe_ = base_in_odom_;
    } else {
        ld_free(curr_ldp_scan);
    }
    last_icp_time_ = time;

    // // 删除prev_ldp_scan_，用curr_ldp_scan进行替代
    // ld_free(pre_ldp_scan_);
    // pre_ldp_scan_ = curr_ldp_scan;
    // last_icp_time_ = time;
}

bool
PLICPOdom::getBaseToLaserTf(const std::string & frame_id)
{
    geometry_msgs::msg::TransformStamped tf2_stamp;

    try {
        tf2_stamp = tf2_buffer_->lookupTransform(base_frame_, frame_id, this->now(), rclcpp::Duration(1, 0));
    } catch(const std::exception& e) {
        std::cerr << e.what() << '\n';
        return false;
    }
    // 将获取到的tf保存到tf中
    tf2::fromMsg(tf2_stamp.transform, base_to_laser_);
    laser_to_base_ = base_to_laser_.inverse();
    
    return true;
    
}

void
PLICPOdom::getPrediction(double & prediction_change_x, double & prediction_change_y, double & prediction_change_angle, double dt)
{
    // 推测从上次ICP的时间到当前时刻的坐标变换
    // 使用匀速模型。根据 当前的速度 * 时间 = 预测的位姿变化
    prediction_change_x = last_velocity_.linear.x < 1e-6 ? 0.0f : last_velocity_.linear.x * dt;
    prediction_change_y = last_velocity_.linear.y < 1e-6 ? 0.0f : last_velocity_.linear.y * dt;
    prediction_change_angle = last_velocity_.angular.z < 1e-6 ? 0.0f : last_velocity_.angular.z * dt;

    if (prediction_change_angle >= M_PI) {
        prediction_change_angle -= 2.0f * M_PI;
    } else if (prediction_change_angle < -M_PI) {
        prediction_change_angle += 2.0f * M_PI;
    }

}

void
PLICPOdom::createTfFromXYTheta(double x, double y, double theta, tf2::Transform& t)
{
    t.setOrigin(tf2::Vector3(x, y, 0.0f));
    tf2::Quaternion q;
    q.setRPY(0.0f, 0.0f, theta);
    t.setRotation(q);
}


/**
 * 如果平移大于阈值，角度大于阈值，则创新新的关键帧
 * @return 需要创建关键帧返回true, 否则返回false
 */
bool
PLICPOdom::NewKeyframeNeeded(const tf2::Transform &d)
{
    scan_count_++;

    if (fabs(tf2::getYaw(d.getRotation())) > kf_dist_angular_)
        return true;

    if (scan_count_ == kf_scan_count_) {
        scan_count_ = 0;
        return true;
    }
        
    double x = d.getOrigin().getX();
    double y = d.getOrigin().getY();
    if (x * x + y * y > kf_dist_linear_sq_)
        return true;

    return false;
}



}  // namespace plicp_odom
