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
#include "iterative_closest_point/iterative_closest_point.hpp"

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
ICP::converMapToPcl()
{
     
}

void
ICP::converScanToPcl()
{

}


void
ICP::processICP()
{
     icp_.setInputSource(source_pointcloud_);
     icp_.setInputTarget(target_pointcloud_);
     icp_.setMaximumIterations(50);

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
