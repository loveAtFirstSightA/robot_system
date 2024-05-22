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

// 1 ICP算法
// 迭代最近点（Iterative Closest Point, 下简称ICP）算法是一种点云匹配算法。

// 其求解思路为：

// 首先对于一幅点云中的每个点，在另一幅点云中计算匹配点（最近点）
// 极小化匹配点间的匹配误差，计算位姿
// 然后将计算的位姿作用于点云
// 再重新计算匹配点
// 如此迭代，直到迭代次数达到阈值，或者极小化函数的变化量小于设定阈值
// ICP算法思路很简单，这里不再进行过多介绍，不了解这个算法的读者可以自行百度一下．

// 2 为什么主流SLAM里不用ICP做帧间匹配
// ICP算法的思路很简单，但是为什么主流SLAM里不用ICP做帧间匹配呢？

// ICP算法有一些不足：

// 首先，ICP对初值比较敏感，初值给的不好，就需要花费更多的迭代次数进行匹配．

// 其次，由于它是迭代很多次的，所以其花费的时间很长，这一点是非常致命的，之后我会通过程序来让大家体验ICP的费时．

// 再次，精度与速度是矛盾的，ICP算法理论上可以实现很高的精度，但是要很多很多的迭代次数，以及很长的时间．所以，当限制了迭代次数的情况下，精度就不一定能保证了．

// 接下来，我将通过代码带着大家感受一下ICP的不足，感受其固定迭代次数时的精度差，以及耗时长．

#include <memory>
#include <chrono>
#include "robot_iterative_closest_point/iterative_closest_point.hpp"

namespace iterative_closest_point
{

IterativeClosestPoint::IterativeClosestPoint()
: Node("iterative_closest_point") 
{
     laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
          // "/bcr_bot/scan",
          "scan",
          1,
          std::bind(&IterativeClosestPoint::laserScanSubCallback, this, std::placeholders::_1));
     
     // 智能指针初始化
     current_pointcloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
     last_pointcloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
}

IterativeClosestPoint::~IterativeClosestPoint()
{

}


void
IterativeClosestPoint::laserScanSubCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{    
     // 记录当前时间
     std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
     // 数据转换
     // 首帧数据单独处理
     if (is_first_scan_) {
          // 首帧数据制作转换，保存
          converScanToPcl(msg);
          is_first_scan_ = false;
          return;
     } else {
          *last_pointcloud_ = *current_pointcloud_;
     }
     // 数据个数转换
     converScanToPcl(msg);
     // 记录转化结束时间
     std::chrono::steady_clock::time_point end_time = std::chrono::steady_clock::now();
     std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(end_time - start_time);
     std::cout << "\n转换数据格式用时: " << time_used.count() << " 秒。" << std::endl;

     // icp计算两帧坐标变换
     start_time = std::chrono::steady_clock::now();
     scanMatchWithICP();
     end_time = std::chrono::steady_clock::now();
     time_used = std::chrono::duration_cast<std::chrono::duration<double>>(end_time - start_time);
     std::cout << "ICP计算用时: " << time_used.count() << " 秒。" << std::endl;

}

/**
 * @brief 将传感器消息中的激光雷达数据转换为 pcl 格式的点云数据
 * 
 * @param scan 传感器消息中的激光雷达数据
 */
void IterativeClosestPoint::converScanToPcl(const sensor_msgs::msg::LaserScan::SharedPtr &scan)
{
    // 创建 pcl 格式的点云指针
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_tmp = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    // 清空点云数据
    point_tmp->clear();
    // 设置点云大小为雷达数据大小
    point_tmp->resize(scan->ranges.size());
    // 创建 pcl 点变量
    pcl::PointXYZ pcl_point;
    
    // 遍历雷达数据
    for (size_t i = 0; i < scan->ranges.size(); i++) {
        float range = scan->ranges[i];
        // 提出无效的数据
        if (!std::isfinite(range)) {
            continue;
        }
        // 判断数据是否在有效范围内
        if (range > scan->range_min && range < scan->range_max) {
            // 计算点的极坐标角度
            float angle = scan->angle_min + scan->angle_increment * i;
            // 将极坐标转换为笛卡尔坐标
            pcl_point.x = range * std::cos(angle);
            pcl_point.y = range * std::sin(angle);
            pcl_point.z = 0.0f;
            // 将点添加到点云中
            point_tmp->push_back(pcl_point);
        }
    }
    
    // 设置点云的宽度、高度和是否密集等属性
    point_tmp->width = point_tmp->size();
    point_tmp->height = 1;
    point_tmp->is_dense = true;
    
    // 将传感器消息的 header 转换为 pcl 格式的 header
    pcl_conversions::toPCL(scan->header, point_tmp->header);
    // 将转换后的点云赋值给当前点云数据
    *current_pointcloud_ = *point_tmp;
}

void
IterativeClosestPoint::scanMatchWithICP()
{
     icp_.setInputSource(last_pointcloud_);
     icp_.setInputTarget(current_pointcloud_);
     icp_.setMaximumIterations(50);

     pcl::PointCloud<pcl::PointXYZ> unused_result;
     icp_.align(unused_result);

     if (icp_.hasConverged() == false) {
          std::cout << "not Converged" << std::endl;
          return;
     } else {
          // 收敛了之后, 获取坐标变换
          Eigen::Affine3f transfrom;
          transfrom = icp_.getFinalTransformation();

          // 将Eigen::Affine3f转换成x, y, theta, 并打印出来
          float x, y, z, roll, pitch, yaw;
          pcl::getTranslationAndEulerAngles(transfrom, x, y, z, roll, pitch, yaw);
          std::cout << "transfrom: (" << x << ", " << y << ", " << yaw * 180 / M_PI << ")" << std::endl;
     }
}






}  // namespace iterative_closest_point
