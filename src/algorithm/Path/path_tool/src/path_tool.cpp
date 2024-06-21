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
     path_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "paths_net",
        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
}

PathTool::~PathTool() {}

void PathTool::timerCallback()
{
     initPath();
     displayCurveOnRviz2();
     sendPathToAlgorithm();
     timer_->cancel();
}

void PathTool::initPath()
{
     // 初始化路径信息
     unsigned int path_nte_type = 0;
     // unsigned int path_nte_type = 1;
     switch (path_nte_type) {
          case 0: {
               // 闭环 直线
               path_.segments.resize(4);
               path_.segments[0].type = path_.segments[0].LINE;
               path_.segments[0].line.p0.x = 1.7701;
               path_.segments[0].line.p0.y = -0.2498;
               path_.segments[0].line.p1.x = -3.7087;
               path_.segments[0].line.p1.y = -0.2661;

               path_.segments[1].type = path_.segments[1].LINE;
               path_.segments[1].line.p0.x = -3.7087;
               path_.segments[1].line.p0.y = -0.2661;
               path_.segments[1].line.p1.x = -3.7731;
               path_.segments[1].line.p1.y = -9.3514;

               path_.segments[2].type = path_.segments[2].LINE;
               path_.segments[2].line.p0.x = -3.7731;
               path_.segments[2].line.p0.y = -9.3514;
               path_.segments[2].line.p1.x = 1.6945;
               path_.segments[2].line.p1.y = -9.4431;

               path_.segments[3].type = path_.segments[3].LINE;
               path_.segments[3].line.p0.x = 1.6945;
               path_.segments[3].line.p0.y = -9.4431;
               path_.segments[3].line.p1.x = 1.7701;
               path_.segments[3].line.p1.y = -0.2498;
          }break;
          case 1: {
               // 闭环 直线+贝塞尔
               // 初始化path_
               path_.segments.resize(8);
               // 按逆时针顺序添加路径段
               // 直线: [0.865, 4.495] -> [0.865, 6.816]
               path_.segments[0].type = path_.segments[0].LINE;
               path_.segments[0].line.p0.x = 0.865;
               path_.segments[0].line.p0.y = 4.495;
               path_.segments[0].line.p1.x = 0.865;
               path_.segments[0].line.p1.y = 6.816;

               // 直线: [0.865, 6.816] -> [-3.756, 6.815]
               path_.segments[1].type = path_.segments[1].LINE;
               path_.segments[1].line.p0.x = 0.865;
               path_.segments[1].line.p0.y = 6.816;
               path_.segments[1].line.p1.x = -3.756;
               path_.segments[1].line.p1.y = 6.815;

               // 直线: [-3.756, 6.815] -> [-3.756, -6.403]
               path_.segments[2].type = path_.segments[2].LINE;
               path_.segments[2].line.p0.x = -3.756;
               path_.segments[2].line.p0.y = 6.815;
               path_.segments[2].line.p1.x = -3.756;
               path_.segments[2].line.p1.y = -6.403;

               // 三阶贝塞尔: [-3.756, -6.403], [-3.756, -9.403], [-3.756, -9.403], [-0.756, -9.403]
               path_.segments[3].type = path_.segments[3].BEZIER3;
               path_.segments[3].bezier3.p0.x = -3.756;
               path_.segments[3].bezier3.p0.y = -6.403;
               path_.segments[3].bezier3.p1.x = -3.756;
               path_.segments[3].bezier3.p1.y = -9.403;
               path_.segments[3].bezier3.p2.x = -3.756;
               path_.segments[3].bezier3.p2.y = -9.403;
               path_.segments[3].bezier3.p3.x = -0.756;
               path_.segments[3].bezier3.p3.y = -9.403;

               // 直线: [-0.756, -9.403] -> [-1.232, -9.403]
               path_.segments[4].type = path_.segments[4].LINE;
               path_.segments[4].line.p0.x = -0.756;
               path_.segments[4].line.p0.y = -9.403;
               path_.segments[4].line.p1.x = -1.232;
               path_.segments[4].line.p1.y = -9.403;

               // 三阶贝塞尔: [-1.232, -9.403], [1.768, -9.403], [1.768, -9.403], [1.768, -6.403]
               path_.segments[5].type = path_.segments[5].BEZIER3;
               path_.segments[5].bezier3.p0.x = -1.232;
               path_.segments[5].bezier3.p0.y = -9.403;
               path_.segments[5].bezier3.p1.x = 1.768;
               path_.segments[5].bezier3.p1.y = -9.403;
               path_.segments[5].bezier3.p2.x = 1.768;
               path_.segments[5].bezier3.p2.y = -9.403;
               path_.segments[5].bezier3.p3.x = 1.768;
               path_.segments[5].bezier3.p3.y = -6.403;

               // 直线: [1.768, -9.403] -> [1.768, -6.403]
               path_.segments[6].type = path_.segments[6].LINE;
               path_.segments[6].line.p0.x = 1.768;
               path_.segments[6].line.p0.y = -9.403;
               path_.segments[6].line.p1.x = 1.768;
               path_.segments[6].line.p1.y = -6.403;

               // 三阶贝塞尔: [1.768, -0.455], [1.768, 2.02], [0.865, 2.02], [0.865, 4.495]
               path_.segments[7].type = path_.segments[7].BEZIER3;
               path_.segments[7].bezier3.p0.x = 1.768;
               path_.segments[7].bezier3.p0.y = -0.455;
               path_.segments[7].bezier3.p1.x = 1.768;
               path_.segments[7].bezier3.p1.y = 2.02;
               path_.segments[7].bezier3.p2.x = 0.865;
               path_.segments[7].bezier3.p2.y = 2.02;
               path_.segments[7].bezier3.p3.x = 0.865;
               path_.segments[7].bezier3.p3.y = 4.495;
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
     auto msg = visualization_msgs::msg::Marker();
     msg.header.frame_id = "map"; // 根据您的实际frame_id进行设置
     msg.header.stamp = this->now(); // 当前时间
     msg.ns = "path";
     msg.id = 0;
     msg.type = visualization_msgs::msg::Marker::LINE_STRIP;
     msg.action = visualization_msgs::msg::Marker::ADD;

     // 设置Marker的比例和颜色
     msg.scale.x = 0.05; // 线宽
     msg.color.r = 0.0; // 红色
     msg.color.g = 1.0; // 绿色
     msg.color.b = 0.0; // 蓝色
     msg.color.a = 1.0; // 透明度

     // 从path_中提取路径的各个点
     for (const auto& segment : path_.segments)
     {
          if (segment.type == segment.LINE)
          {
               geometry_msgs::msg::Point p0;
               p0.x = segment.line.p0.x;
               p0.y = segment.line.p0.y;
               p0.z = 0.0;
               msg.points.push_back(p0);

               geometry_msgs::msg::Point p1;
               p1.x = segment.line.p1.x;
               p1.y = segment.line.p1.y;
               p1.z = 0.0;
               msg.points.push_back(p1);
          }
          else if (segment.type == segment.BEZIER3)
          {
               // 添加贝塞尔曲线的控制点
               geometry_msgs::msg::Point p0;
               p0.x = segment.bezier3.p0.x;
               p0.y = segment.bezier3.p0.y;
               p0.z = 0.0;
               msg.points.push_back(p0);

               geometry_msgs::msg::Point p1;
               p1.x = segment.bezier3.p1.x;
               p1.y = segment.bezier3.p1.y;
               p1.z = 0.0;
               msg.points.push_back(p1);

               geometry_msgs::msg::Point p2;
               p2.x = segment.bezier3.p2.x;
               p2.y = segment.bezier3.p2.y;
               p2.z = 0.0;
               msg.points.push_back(p2);

               geometry_msgs::msg::Point p3;
               p3.x = segment.bezier3.p3.x;
               p3.y = segment.bezier3.p3.y;
               p3.z = 0.0;
               msg.points.push_back(p3);
          }
          else if (segment.type == segment.BEZIER5)
          {
               // 添加五阶贝塞尔曲线的控制点
               geometry_msgs::msg::Point p0;
               p0.x = segment.bezier5.p0.x;
               p0.y = segment.bezier5.p0.y;
               p0.z = 0.0;
               msg.points.push_back(p0);

               geometry_msgs::msg::Point p1;
               p1.x = segment.bezier5.p1.x;
               p1.y = segment.bezier5.p1.y;
               p1.z = 0.0;
               msg.points.push_back(p1);

               geometry_msgs::msg::Point p2;
               p2.x = segment.bezier5.p2.x;
               p2.y = segment.bezier5.p2.y;
               p2.z = 0.0;
               msg.points.push_back(p2);

               geometry_msgs::msg::Point p3;
               p3.x = segment.bezier5.p3.x;
               p3.y = segment.bezier5.p3.y;
               p3.z = 0.0;
               msg.points.push_back(p3);

               geometry_msgs::msg::Point p4;
               p4.x = segment.bezier5.p4.x;
               p4.y = segment.bezier5.p4.y;
               p4.z = 0.0;
               msg.points.push_back(p4);

               geometry_msgs::msg::Point p5;
               p5.x = segment.bezier5.p5.x;
               p5.y = segment.bezier5.p5.y;
               p5.z = 0.0;
               msg.points.push_back(p5);
          }
     }

     // 发布Marker消息
     path_marker_pub_->publish(msg);
}

void PathTool::sendPathToAlgorithm()
{
     auto msg = algorithm_msgs::msg::Path();
     msg = path_;
     path_pub_->publish(msg);
}


}  // namespace path_tool
