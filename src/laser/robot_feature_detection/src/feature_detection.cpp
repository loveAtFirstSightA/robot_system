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

#include <chrono>
#include <map>
#include "robot_feature_detection/feature_detection.hpp"

namespace feature_detection
{
FeatureDetection::FeatureDetection()
: Node("feature_detection")
{
     laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
          // "/bcr_bot/scan",
          "scan",
          1,
          std::bind(&FeatureDetection::laserScanSubCallback, this, std::placeholders::_1));
     featue_scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
          "/feature_scan",
          1);
     
     // 将提取角点的阈值设置为1.0
     edge_threshold_ = 1.0;
}

FeatureDetection::~FeatureDetection()
{
    
}

// #define max_scan_count 361
void
FeatureDetection::laserScanSubCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
     unsigned int max_scan_count = msg->ranges.size();
     std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
     std::vector<smoothness_t> scan_smoothness_(max_scan_count);   // 存储每个点的曲率和索引
     // 动态内存分配 
     // 使用new分配的内存是在堆上分配的。这与在栈上分配的内存（例如普通的局部变量）不同。
     // 堆上的内存在程序运行时手动管理，可以在函数返回后仍然存在，直到显式释放。
     // 如何释放 delete[] scan_curvature_;
     float * scan_curvature_ = new float[max_scan_count];   //   存储每个点的曲率

     std::map<int, int> map_index; //   有效点的索引对应scan实际的索引号
     int count = 0; //   有效点的索引
     float new_scan[max_scan_count];    //   存储scan数据的距离数值

     int scan_count = msg->ranges.size();    //   通过ranges中点的个数对雷达点进行遍历

     // 去除inf 和 nan点
     for (int i = 0; i < scan_count; i ++) {
          // 检查一个浮点数是否是有限的
          if (!std::isfinite(msg->ranges[i])) {
               continue;
          }
          map_index[count] = i;    //   这些点在原始数据的索引为i 在new_scan中索引为count
          new_scan[count] = msg->ranges[i];  //   在new_scan中保存有效点的距离数值
          count ++;
     }

     // 计算曲率值, 通过当前点前后5个点距离值的偏差程度来代表曲率
     // 如果是球面, 则当前点周围的10个点的距离之和 减去 当前点距离的10倍 应该等于0
     for (int i = 5; i < count - 5; i ++) {
          // 计算当前点前后5个点的距离之和与当前点距离值的10倍的差值
          float diff_range = new_scan[i - 5] + new_scan[i - 4] + 
                              new_scan[i - 3] + new_scan[i - 2] + 
                              new_scan[i - 1] - new_scan[i] * 10 +
                              new_scan[i + 1] + new_scan[i + 2] +
                              new_scan[i + 3] + new_scan[i + 4] +
                              new_scan[i + 5];
          // diffX * diffX + diffY * diffY
          //    曲率值 scan_curvature_[i] 是 diff_range 的平方。这是为了确保曲率值始终为非负值。
          scan_curvature_[i] = diff_range * diff_range;
          scan_smoothness_[i].value = scan_curvature_[i];
          scan_smoothness_[i].index = i;
     }
     // 声明一个临时的sensor_msgs::LaserScan变量,用于存储特征提取后的scan数据,并发布出去,在rviz中进行显示
     sensor_msgs::msg::LaserScan corner_scan;
     corner_scan.header = msg->header;
     corner_scan.angle_min = msg->angle_min;
     corner_scan.angle_max = msg->angle_max;
     corner_scan.angle_increment = msg->angle_increment;
     corner_scan.range_min = msg->range_min;
     corner_scan.range_max = msg->range_max;

     // 对float进行初始化
     corner_scan.ranges.resize(max_scan_count);

     // 进行角点的提取,将完整的scan分成6部分,每部分提取20个角点
     // 从激光扫描数据中提取角点（高曲率点）。具体来说，它将完整的扫描数据分成6个部分，并从每个部分中提取出20个角点。
     // 首先，为了保证特征点分布的更均匀，将scan数据分成6部分，每个部分取最多20个特征点。
     // 先计算一下这1/6数据段的起始点和终止点的索引，再将这段数据根据曲率值由小到大进行排序。
     // 这样曲率值大的点处在后面的部分，我们通过从后往前的遍历，选取最多20个点，并将这些点填充进新声明的corner_scan中。
     // 输入的索引总数是count
     for (int j = 0; j < 6; j ++) {
          int start_index = (0 * (6 - j) + count * j) / 6;
          int end_index = (0 * (5 - j) + count * (j + 1)) / 6 - 1;
          std::cout << "start_index: " << start_index << " end_index: " << end_index << std::endl;

          if (start_index >= end_index)
               continue;

          // 将这段点云按照曲率从小到大进行排序
          std::sort(scan_smoothness_.begin() + start_index,
                    scan_smoothness_.begin() + end_index, by_value());

          int largestPickedNum = 0;
          // 最后的点 的曲率最大，如果满足条件，就是角点
          for (int k = end_index; k >= start_index; k--) {
               int index = scan_smoothness_[k].index;
               if (scan_smoothness_[k].value > edge_threshold_) {
                    // 每一段最多只取20个角点
                    largestPickedNum ++;
                    if (largestPickedNum <= 20) {
                         corner_scan.ranges[map_index[index]] = msg->ranges[map_index[index]];
                    }
                    else {
                         break;
                    }
               }
          }
     }

     featue_scan_pub_->publish(corner_scan);

     std::chrono::steady_clock::time_point end_time = std::chrono::steady_clock::now();
     std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(end_time - start_time);
     std::cout << "处理一次数据用时: " << time_used.count() << " 秒。" << std::endl;
}




}  // namespace feature_detection
