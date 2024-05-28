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

#include "iterative_closest_point/scan_to_map_icp.hpp"

namespace iterative_closest_point
{
ScanToMapICP::ScanToMapICP()
: Node("scan_to_map_icp")
{
     map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
          "map",
          rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
          std::bind(&ScanToMapICP::mapSubCallback, this, std::placeholders::_1)); 
     scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
          "scan",
          10,
          std::bind(&ScanToMapICP::scanSubCallback, this, std::placeholders::_1)); 

     // init tf
     tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
     tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);  

}

ScanToMapICP::~ScanToMapICP()
{

}


void
ScanToMapICP::mapSubCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
     RCLCPP_INFO(this->get_logger(), "step 1 将栅格地图数据转换成PCL数据格式");
     // step 1 将栅格地图数据转换成PCL数据格式
     icp_.converMapToPcl(msg);
}
void
ScanToMapICP::scanSubCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
     geometry_msgs::msg::TransformStamped tf;
     // step 2 获取雷达在地图上的位置
     if (!calculateTfLidarToMap(tf)) {
          RCLCPP_WARN(this->get_logger(), "return");
          return;
     }
     double x = tf.transform.translation.x;
     double y = tf.transform.translation.y;
     double yaw = tf2::getYaw(tf.transform.rotation);
     // step 3 将雷达点变换至地图坐标系
     // step 4 转换激光点为PCL数据格式
     icp_.converScanToPcl(msg, x, y, yaw);
     // step 5 调用PCLMATCH，完成数据迭代
     icp_.processICP();
}

bool
ScanToMapICP::calculateTfLidarToMap(geometry_msgs::msg::TransformStamped & tf) 
{
     std::string target_link = "map";
     std::string source_link = "base_scan";
     // std::string source_link = "laser_2d_link";
     try {
          tf = tf2_buffer_->lookupTransform(target_link, source_link, tf2::TimePointZero);
          RCLCPP_DEBUG(this->get_logger(), "TF %s to %s, [%lf, %lf, %lf]",
               source_link.c_str(), target_link.c_str(), tf.transform.translation.x, tf.transform.translation.y, tf2::getYaw(tf.transform.rotation));
          return true;
     } catch (tf2::TransformException & ex) {
          std::cerr << "Could not get transform from " << source_link << " to " << target_link << ": " << ex.what() << '\n';
          return false;
     }

}







}  // namespace iterative_closest_point
