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

#include "robot_path_creater/path_creater.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace robot_path_creater
{
PathCreater::PathCreater()
: Node("robot_path_creater")
{
    RCLCPP_INFO(this->get_logger(), "robot path creater unit has been executed");
    initPubSub();
    initTimer();
}

PathCreater::~PathCreater()
{
    // 
}

void PathCreater::initPubSub()
{
    markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "paths_net", 10);
    cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
        "cmd_path",
        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
        std::bind(&PathCreater::cmdSubCallback, this, std::placeholders::_1));
    
    path_points_pub_ = this->create_publisher<robot_msgs::msg::Vector3Array>(
        "path_net_points",
        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
}

void PathCreater::cmdSubCallback(const std_msgs::msg::String::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received message");
    if (msg->data == "fix_path") {
        // lode fixed path net
        // 1、加载路网信息
        // 2、解算直线 贝塞尔曲线 
        // 3、确定分辨率 [0-t]
        // 4、计算
    }
}

void PathCreater::displayCurveOnRviz2(std::vector<geometry_msgs::msg::Vector3> points)
{
    auto msg = visualization_msgs::msg::MarkerArray();
    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = "map"; // 设置你的帧ID
    marker.header.stamp = rclcpp::Clock().now();
    marker.ns = "paths_net";
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05; // 设置线的宽度

    // 设置线的颜色
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    // 设置点
    for (const auto& point : points) {
        geometry_msgs::msg::Point p;
        p.x = point.x;
        p.y = point.y;
        p.z = point.z;
        marker.points.push_back(p);
    }
    marker.lifetime = rclcpp::Duration(0, 0);
    msg.markers.push_back(marker);
    markers_pub_->publish(msg);
}

void PathCreater::initTimer()
{
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&PathCreater::timerCallback, this));
}

void PathCreater::timerCallback()
{
    RCLCPP_INFO(this->get_logger(), "Timer event");

    geometry_msgs::msg::Vector3 p0, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11;
    p0.x = 0.865;
    p0.y = 4.495;
    p1.x = 0.865;
    p1.y = 6.816;
    p2.x = -3.756;
    p2.y = 6.815;

    p3.x = -3.756;
    p3.y = -6.403;
    p4.x = -3.756;
    p4.y = -9.403;
    p5.x = -0.756;
    p5.y = -9.403;

    p6.x = -1.232;
    p6.y = -9.403;
    p7.x = 1.768;
    p7.y = -9.403;
    p8.x = 1.768;
    p8.y = -6.403;

    p9.x = 1.768; 
    p9.y = -0.455;
    p10.x = 1.768;
    p10.y = 2.02;
    p11.x = 0.865; 
    p11.y = 2.02;
    
    std::vector<geometry_msgs::msg::Vector3> line0 = calculateLinePoints(p0, p1, 0.001);
    std::vector<geometry_msgs::msg::Vector3> line1 = calculateLinePoints(p1, p2, 0.001);
    std::vector<geometry_msgs::msg::Vector3> line2 = calculateLinePoints(p2, p3, 0.001);
    std::vector<geometry_msgs::msg::Vector3> bezier0 = calculateBezier3Points(p3, p4, p4, p5, 0.001);
    std::vector<geometry_msgs::msg::Vector3> line3 = calculateLinePoints(p5, p6, 0.001);
    std::vector<geometry_msgs::msg::Vector3> bezier1 = calculateBezier3Points(p6, p7, p7, p8, 0.001);
    std::vector<geometry_msgs::msg::Vector3> line4 = calculateLinePoints(p8, p9, 0.001);
    std::vector<geometry_msgs::msg::Vector3> bezier2 = calculateBezier3Points(p9, p10, p11, p0, 0.001);

    std::vector<geometry_msgs::msg::Vector3> points;
    for (size_t i = 0; i < line0.size(); i++) {
        points.push_back(line0[i]);
    }
    for (size_t i = 0; i < line1.size(); i++) {
        points.push_back(line1[i]);
    }
    for (size_t i = 0; i < line2.size(); i++) {
        points.push_back(line2[i]);
    }
    for (size_t i = 0; i < bezier0.size(); i++) {
        points.push_back(bezier0[i]);
    }
    for (size_t i = 0; i < line3.size(); i++) {
        points.push_back(line3[i]);
    }
    for (size_t i = 0; i < bezier1.size(); i++) {
        points.push_back(bezier1[i]);
    }
    for (size_t i = 0; i < line4.size(); i++) {
        points.push_back(line4[i]);
    }
    for (size_t i = 0; i < bezier2.size(); i++) {
        points.push_back(bezier2[i]);
    }
    displayCurveOnRviz2(points);
    // 发布路径点
    auto msg = robot_msgs::msg::Vector3Array();
    for (size_t i = 0; i < points.size(); i++) {
        msg.vector3s.push_back(points.at(i));
    }
    RCLCPP_INFO(this->get_logger(), "Publishing paths paints size %ld", points.size());
    path_points_pub_->publish(msg);

    timer_->cancel();

}


// straight line
std::vector<geometry_msgs::msg::Vector3> 
PathCreater::calculateLinePoints(geometry_msgs::msg::Vector3 start, geometry_msgs::msg::Vector3 end, double t)
{
    geometry_msgs::msg::Vector3 point;
    std::vector<geometry_msgs::msg::Vector3> points;
    geometry_msgs::msg::Vector3 direction = calculateVector(start, end);
    // RCLCPP_INFO(this->get_logger(), "direction [%lf, %lf]", direction.x, direction.y);
    for (double i = 0; i < 1.0; i += t) {
        point.x = start.x + i * direction.x;
        point.y = start.y + i * direction.y;
        points.push_back(point);
    }

    return points;
}

geometry_msgs::msg::Vector3
PathCreater::calculateVector(geometry_msgs::msg::Vector3 p0, geometry_msgs::msg::Vector3 p1)
{
    geometry_msgs::msg::Vector3 vector;
    vector.x = p1.x - p0.x;
    vector.y = p1.y - p0.y;

    return vector;
}

double PathCreater::calculateVectorDist(geometry_msgs::msg::Vector3 p)
{
    return std::sqrt(p.x * p.x + p.y + p.y);
}

// Third-order Bezier curve
// 三阶贝塞尔曲线的数学表达式
/*
 * P(t) = (1 - t)^3 * P_0 + 3 * t * (1 - t)^2 * P_1 + 3 * t^2 * (1 - t) * P_2 + t^3 * P_3
 */
std::vector<geometry_msgs::msg::Vector3>
PathCreater::calculateBezier3Points(geometry_msgs::msg::Vector3 p0, geometry_msgs::msg::Vector3 p1, 
    geometry_msgs::msg::Vector3 p2, geometry_msgs::msg::Vector3 p3, double t)
{
    geometry_msgs::msg::Vector3 point;
    std::vector<geometry_msgs::msg::Vector3> points;
    for (double i = 0.0; i <= 1.0; i += t) {
        point.x = std::pow(1.0 - i, 3) * p0.x + 3.0 * i * std::pow(1 - i, 2) * p1.x + 3.0 * std::pow(i, 2) * (1.0 -i) * p2.x + std::pow(i, 3) * p3.x;
        point.y = std::pow(1.0 - i, 3) * p0.y + 3.0 * i * std::pow(1 - i, 2) * p1.y + 3.0 * std::pow(i, 2) * (1.0 -i) * p2.y + std::pow(i, 3) * p3.y;
        points.push_back(point);
    }

    return points;
}

// 五阶贝塞尔曲线的数学表达式
/*
 * P(t) = (1 - t)^5 * P_0 + 5 * (1 - t)^4 * t * P_1 + 10 * (1 - t)^3 * t^2 * P_2 
 *        + 10 * (1 - t)^2 * t^3 * P_3 + 5 * (1 - t) * t^4 * P_4 + t^5 * P_5
 */
std::vector<geometry_msgs::msg::Vector3>
PathCreater::calculateBezier5Points(geometry_msgs::msg::Vector3 p0, geometry_msgs::msg::Vector3 p1, geometry_msgs::msg::Vector3 p2,
    geometry_msgs::msg::Vector3 p3, geometry_msgs::msg::Vector3 p4, geometry_msgs::msg::Vector3 p5, double t)
{
    geometry_msgs::msg::Vector3 point;
    std::vector<geometry_msgs::msg::Vector3> points;
    for (double i = 0.0; i <= 1.0; i += t) {
        point.x = std::pow(1.0 - i, 5) * p0.x + 5.0 * i * std::pow(1 - i, 4) * p1.x + 10.0 * std::pow(i, 2) * std::pow(1 - i, 3) * p2.x + 10.0 * std::pow(i, 3) * std::pow(1 - i, 2) * p3.x + 5.0 * std::pow(i, 4) * (1 - i) * p4.x + std::pow(i, 5) * p5.x;
        point.y = std::pow(1.0 - i, 5) * p0.y + 5.0 * i * std::pow(1 - i, 4) * p1.y + 10.0 * std::pow(i, 2) * std::pow(1 - i, 3) * p2.y + 10.0 * std::pow(i, 3) * std::pow(1 - i, 2) * p3.y + 5.0 * std::pow(i, 4) * (1 - i) * p4.y + std::pow(i, 5) * p5.y;
        points.push_back(point);
    }

    return points;
}













}  // namespace robot_path_creater
