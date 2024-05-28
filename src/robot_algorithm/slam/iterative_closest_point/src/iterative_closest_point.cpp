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
#include <ctime>
#include <iomanip>
#include "iterative_closest_point/iterative_closest_point.hpp"
#include "iterative_closest_point/logger.hpp"

namespace iterative_closest_point
{
ICP::ICP()
{

}

ICP::~ICP()
{
     // 
}

void
ICP::converMapToPcl(const nav_msgs::msg::OccupancyGrid::SharedPtr map)
{
     unsigned int width = static_cast<unsigned int>(map->info.width);
     unsigned int height = static_cast<unsigned int>(map->info.height);
     double resolution = static_cast<double>(map->info.resolution);
     double origin_x = map->info.origin.position.x;
     double origin_y = map->info.origin.position.y;
     std::cout << getCurrentTime() << "map: " << width << " * " << height << " " << resolution << " [" << origin_x << " ," << origin_y << "]" << std::endl;
     // map data_status occ - 100 unknown - -1 free - 0
     // 栅格地图的info原点是相对于哪里的？
     std::cout << getCurrentTime() << "开始转换地图数据" << std::endl;
     // 障碍点个数
     unsigned int sum_occ = 0;
     std::vector<double> occ_x;
     std::vector<double> occ_y;
     occ_x.clear();
     occ_y.clear();
     for (size_t i = 0; i < map->data.size(); i++) {
          // std::cout << getCurrentTime() << i << ": " << static_cast<int>(map->data[i]) << std::endl;
          if (map->data[i] == 100) {
               sum_occ ++;
               // 提取地图坐标下每个障碍点的位置
               // 计算栅格的行序 和 列序
               int row = i / width;     //   行序号
               int col = i % width;     //   列序号
               // 计算实际位姿
               double x = origin_x + row * resolution;
               double y = origin_y + col * resolution;
               occ_x.push_back(x);
               occ_y.push_back(y);
          }
     }
     // convert pcl
     // 创建 pcl 格式的点云指针
     pcl::PointCloud<pcl::PointXYZ>::Ptr point_tmp = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
     // 清空点云数据
     point_tmp->clear();
     // 设置点云大小为障碍点个数大小
     point_tmp->resize(sum_occ);
     // 创建 pcl 点变量
     pcl::PointXYZ pcl_point;
     // 遍历occ数据
     for (size_t i = 0; i < sum_occ; i++) {
          pcl_point.x = occ_x[i];
          pcl_point.y = occ_y[i];
          pcl_point.z = 0.0f;
          point_tmp->push_back(pcl_point);
          point_tmp->push_back(pcl_point);
     }
     
     // 设置点云的宽度、高度和是否密集等属性
     point_tmp->width = point_tmp->size();
     point_tmp->height = 1;
     point_tmp->is_dense = true;

     // 将传感器消息的 header 转换为 pcl 格式的 header
     pcl_conversions::toPCL(map->header, point_tmp->header);
     target_pointcloud_ = point_tmp;
}

void
ICP::converScanToPcl(const sensor_msgs::msg::LaserScan::SharedPtr scan, double x, double y, double yaw)
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
               float lx = range * std::cos(angle);
               float ly = range * std::sin(angle);

               // 考虑坐标系变换 雷达坐标系变换至地图坐标系
               float mx = lx * std::cos(yaw) - ly * std::sin(yaw) + x;
               float my = lx * std::sin(yaw) + ly * std::cos(yaw) + y;

               // 将极坐标转换为笛卡尔坐标
               pcl_point.x = mx;
               pcl_point.y = my;
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
    source_pointcloud_ = point_tmp;
}


void
ICP::processICP()
{
     icp_.setInputSource(source_pointcloud_);
     icp_.setInputTarget(target_pointcloud_);
     icp_.setMaximumIterations(80);

     pcl::PointCloud<pcl::PointXYZ> unused_result;
     icp_.align(unused_result);

     if (icp_.hasConverged() == false) {
          std::cout << "Not Converged" << std::endl;
          return;
     } else {
          // 收敛了之后, 获取坐标变换
          Eigen::Affine3f transfrom;
          transfrom = icp_.getFinalTransformation();
          // 将Eigen::Affine3f转换成x, y, theta, 并打印出来
          float x, y, z, roll, pitch, yaw;
          pcl::getTranslationAndEulerAngles(transfrom, x, y, z, roll, pitch, yaw);
          std::cout << "transfrom: (" << x << ", " << y << ", " << yaw << ")" << std::endl;
     }
}








}  // namespace iterative_closest_point
