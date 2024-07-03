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

#include "path_tool/path_tool.hpp"

namespace path_tool
{
PathTool::PathTool() : Node ("path_tool")
{
     timer_ = this->create_wall_timer(
          std::chrono::seconds(1),
          std::bind(&PathTool::timerCallback, this));
     path_pub_ = this->create_publisher<algorithm_msgs::msg::Path>(
          "algorithm_path",
          rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
     path_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "paths_net",
        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
}

PathTool::~PathTool() {}

void PathTool::timerCallback()
{    
     // step 1 初始化路网
     initPath();
     // step 2 发布路网信息
     sendPathToAlgorithm();
     // step 3 离散化路网信息为短直线段
     convertPathToPoints();
     // step 4 显示路网
     displayCurveOnRviz2();
     // step 5 取消定时器
     // timer_->cancel();
}

void PathTool::initPath()
{
     // 初始化路径信息
     // unsigned int path_nte_type = 0;
     unsigned int path_nte_type = 1;
     // unsigned int path_nte_type = 2;
     switch (path_nte_type) {
          case 0: {
               // 闭环 直线
               path_.segments.resize(4);
               path_.segments[0].type = path_.segments[0].LINE;
               path_.segments[0].line.p0.x = 2.0;
               path_.segments[0].line.p0.y = 0.0;
               path_.segments[0].line.p1.x = -4.0;
               path_.segments[0].line.p1.y = 0.0;

               path_.segments[1].type = path_.segments[1].LINE;
               path_.segments[1].line.p0.x = -4.0;
               path_.segments[1].line.p0.y = 0.0;
               path_.segments[1].line.p1.x = -4.0;
               path_.segments[1].line.p1.y = -9.5;

               path_.segments[2].type = path_.segments[2].LINE;
               path_.segments[2].line.p0.x = -4.0;
               path_.segments[2].line.p0.y = -9.5;
               path_.segments[2].line.p1.x = 2.0;
               path_.segments[2].line.p1.y = -9.5;

               path_.segments[3].type = path_.segments[3].LINE;
               path_.segments[3].line.p0.x = 2.0;
               path_.segments[3].line.p0.y = -9.5;
               path_.segments[3].line.p1.x = 2.0;
               path_.segments[3].line.p1.y = 0.0;
          }break;
          case 1: {
               // 闭环 直线+贝塞尔
               // 初始化path_
               path_.segments.resize(10);
               // 按逆时针顺序添加路径段
               path_.segments[0].type = path_.segments[0].LINE;
               path_.segments[0].line.p0.x = 1.0;
               path_.segments[0].line.p0.y = 4.0;
               path_.segments[0].line.p1.x = 1.0;
               path_.segments[0].line.p1.y = 6.0;

               path_.segments[1].type = path_.segments[1].BEZIER3;
               path_.segments[1].bezier3.p0.x = 1.0;
               path_.segments[1].bezier3.p0.y = 6.0;
               path_.segments[1].bezier3.p1.x = 1.0;
               path_.segments[1].bezier3.p1.y = 6.5;
               path_.segments[1].bezier3.p2.x = 0.5;
               path_.segments[1].bezier3.p2.y = 7.0;
               path_.segments[1].bezier3.p3.x = 0.0;
               path_.segments[1].bezier3.p3.y = 7.0;

               path_.segments[2].type = path_.segments[2].LINE;
               path_.segments[2].line.p0.x = 0.0;
               path_.segments[2].line.p0.y = 7.0;
               path_.segments[2].line.p1.x = -3.0;
               path_.segments[2].line.p1.y = 7.0;

               path_.segments[3].type = path_.segments[3].BEZIER3;
               path_.segments[3].bezier3.p0.x = -3.0;
               path_.segments[3].bezier3.p0.y = 7.0;
               path_.segments[3].bezier3.p1.x = -3.5;
               path_.segments[3].bezier3.p1.y = 7.0;
               path_.segments[3].bezier3.p2.x = -4.0;
               path_.segments[3].bezier3.p2.y = 6.5;
               path_.segments[3].bezier3.p3.x = -4.0;
               path_.segments[3].bezier3.p3.y = 6.0;

               path_.segments[4].type = path_.segments[4].LINE;
               path_.segments[4].line.p0.x = -4.0;
               path_.segments[4].line.p0.y = 6.0;
               path_.segments[4].line.p1.x = -4.0;
               path_.segments[4].line.p1.y = -7.0;

               path_.segments[5].type = path_.segments[5].BEZIER3;
               path_.segments[5].bezier3.p0.x = -4.0;
               path_.segments[5].bezier3.p0.y = -7.0;
               path_.segments[5].bezier3.p1.x = -4.0;
               path_.segments[5].bezier3.p1.y = -9.0;
               path_.segments[5].bezier3.p2.x = -3.0;
               path_.segments[5].bezier3.p2.y = -9.5;
               path_.segments[5].bezier3.p3.x = -1.5;
               path_.segments[5].bezier3.p3.y = -9.5;

               path_.segments[6].type = path_.segments[6].LINE;
               path_.segments[6].line.p0.x = -1.5;
               path_.segments[6].line.p0.y = -9.5;
               path_.segments[6].line.p1.x = 0.0;
               path_.segments[6].line.p1.y = -9.5;

               path_.segments[7].type = path_.segments[7].BEZIER3;
               path_.segments[7].bezier3.p0.x = 0.0;
               path_.segments[7].bezier3.p0.y = -9.5;
               path_.segments[7].bezier3.p1.x = 1.5;
               path_.segments[7].bezier3.p1.y = -9.5;
               path_.segments[7].bezier3.p2.x = 2.0;
               path_.segments[7].bezier3.p2.y = -9.0;
               path_.segments[7].bezier3.p3.x = 2.0;
               path_.segments[7].bezier3.p3.y = -7.5;

               path_.segments[8].type = path_.segments[8].LINE;
               path_.segments[8].line.p0.x = 2.0;
               path_.segments[8].line.p0.y = -7.5;
               path_.segments[8].line.p1.x = 2.0;
               path_.segments[8].line.p1.y = 0.0;

               path_.segments[9].type = path_.segments[9].BEZIER3;
               path_.segments[9].bezier3.p0.x = 2.0;
               path_.segments[9].bezier3.p0.y = 0.0;
               path_.segments[9].bezier3.p1.x = 2.0;
               path_.segments[9].bezier3.p1.y = 2.0;
               path_.segments[9].bezier3.p2.x = 1.0;
               path_.segments[9].bezier3.p2.y = 2.0;
               path_.segments[9].bezier3.p3.x = 1.0;
               path_.segments[9].bezier3.p3.y = 4.0;
          }break;
          case 2: {
               // 闭环 直线来回
               path_.segments.resize(2);
               path_.segments[0].type = path_.segments[0].LINE;
               path_.segments[0].line.p0.x = 0.0;
               path_.segments[0].line.p0.y = -0.25;
               path_.segments[0].line.p1.x = -4.0;
               path_.segments[0].line.p1.y = -0.25;

               path_.segments[1].type = path_.segments[1].LINE;
               path_.segments[1].line.p0.x = -4.0;
               path_.segments[1].line.p0.y = -0.25;
               path_.segments[1].line.p1.x = 0.0;
               path_.segments[1].line.p1.y = -0.25;
          }break;
          default: {
               std::cout << "Unknown path net type" << std::endl;
               return;
          }break;
     }
}

void PathTool::displayCurveOnRviz2()
{
     // 创建Marker消息
     auto msg = visualization_msgs::msg::MarkerArray();
     visualization_msgs::msg::Marker marker;
     marker.header.frame_id = "map"; // 根据您的实际frame_id进行设置
     marker.header.stamp = this->now(); // 当前时间
     marker.ns = "path";
     marker.id = 0;
     marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
     marker.action = visualization_msgs::msg::Marker::ADD;

     // 设置Marker的比例和颜色
     marker.scale.x = 0.05; // 线宽
     marker.color.r = 0.0; // 红色
     marker.color.g = 1.0; // 绿色
     marker.color.b = 0.0; // 蓝色
     marker.color.a = 1.0; // 透明度

     for (const auto& segment_points : paths_points_) {
          for (const auto& point : segment_points) {
               geometry_msgs::msg::Point p;
               p.x = point.x;
               p.y = point.y;
               p.z = point.z; // 假设paths_points_中的点包含z坐标
               marker.points.push_back(p);
          }
     }

     // 将Marker添加到MarkerArray
     msg.markers.push_back(marker);

     // 发布Marker消息
     path_marker_pub_->publish(msg);
}

void PathTool::sendPathToAlgorithm()
{
     auto msg = algorithm_msgs::msg::Path();
     msg = path_;
     path_pub_->publish(msg);
}

void PathTool::convertPathToPoints()
{
     paths_points_.clear();
     paths_points_.resize(path_.segments.size());

     for (size_t i = 0; i < path_.segments.size(); i++) {
          if (path_.segments[i].type == path_.segments[i].LINE) {
               convertLineToPoints(path_.segments[i].line, paths_points_[i]);
          }
          if (path_.segments[i].type == path_.segments[i].BEZIER3) {
               convertBezier3ToPoints(path_.segments[i].bezier3, paths_points_[i]);
          }
          if (path_.segments[i].type == path_.segments[i].BEZIER5) {
               convertBezier5ToPoints(path_.segments[i].bezier5, paths_points_[i]);
          }
     }
}



void PathTool::convertLineToPoints(const algorithm_msgs::msg::Line& line, std::vector<geometry_msgs::msg::Point>& points, double step)
{
     double dx = line.p1.x - line.p0.x;
     double dy = line.p1.y - line.p0.y;
     double length = sqrt(dx * dx + dy * dy);
     int num_steps = static_cast<int>(length / step);

     for (int i = 0; i <= num_steps; ++i) {
          geometry_msgs::msg::Point point;
          point.x = line.p0.x + dx * i / num_steps;
          point.y = line.p0.y + dy * i / num_steps;
          point.z = 0; // 假设z坐标为0，因为algorithm_msgs::msg::Point结构中没有z
          points.push_back(point);
     }
}

void PathTool::convertBezier3ToPoints(const algorithm_msgs::msg::Bezier3& bezier, std::vector<geometry_msgs::msg::Point>& points, double step)
{
     for (double t = 0.0; t <= 1.0; t += step) {
          geometry_msgs::msg::Point point;
          double u = 1 - t;
          point.x = u * u * u * bezier.p0.x + 3 * u * u * t * bezier.p1.x + 3 * u * t * t * bezier.p2.x + t * t * t * bezier.p3.x;
          point.y = u * u * u * bezier.p0.y + 3 * u * u * t * bezier.p1.y + 3 * u * t * t * bezier.p2.y + t * t * t * bezier.p3.y;
          point.z = 0; // 假设z坐标为0，因为algorithm_msgs::msg::Point结构中没有z
          points.push_back(point);
     }
}

void PathTool::convertBezier5ToPoints(const algorithm_msgs::msg::Bezier5& bezier, std::vector<geometry_msgs::msg::Point>& points, double step)
{
     for (double t = 0.0; t <= 1.0; t += step) {
          geometry_msgs::msg::Point point;
          double u = 1 - t;
          point.x = pow(u, 5) * bezier.p0.x + 5 * pow(u, 4) * t * bezier.p1.x + 10 * pow(u, 3) * t * t * bezier.p2.x + 10 * pow(u, 2) * t * t * t * bezier.p3.x + 5 * u * t * t * t * t * bezier.p4.x + t * t * t * t * t * bezier.p5.x;
          point.y = pow(u, 5) * bezier.p0.y + 5 * pow(u, 4) * t * bezier.p1.y + 10 * pow(u, 3) * t * t * bezier.p2.y + 10 * pow(u, 2) * t * t * t * bezier.p3.y + 5 * u * t * t * t * t * bezier.p4.y + t * t * t * t * t * bezier.p5.y;
          point.z = 0; // 假设z坐标为0，因为algorithm_msgs::msg::Point结构中没有z
          points.push_back(point);
     }
}


}  // namespace path_tool
