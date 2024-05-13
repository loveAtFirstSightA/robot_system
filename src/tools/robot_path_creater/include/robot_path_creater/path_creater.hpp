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

#ifndef ROBOT_PATH_CREATER__PATH_CREATER_HPP_
#define ROBOT_PATH_CREATER__PATH_CREATER_HPP_

#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "robot_msgs/msg/pose.hpp"
#include "robot_msgs/msg/pose_array.hpp"

namespace robot_path_creater
{
class PathCreater : public rclcpp::Node
{
public:
    PathCreater();
    ~PathCreater();

private:
    void initPubSub();
    void initTimer();
    void cmdSubCallback(const std_msgs::msg::String::SharedPtr msg);
    void timerCallback();
    void displayCurveOnRviz2(std::vector<geometry_msgs::msg::Vector3> points);

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

private:
    // math area
    std::vector<geometry_msgs::msg::Vector3>
        calculateLinePoints(geometry_msgs::msg::Vector3 start, geometry_msgs::msg::Vector3 end, double t);
    geometry_msgs::msg::Vector3 calculateVector(geometry_msgs::msg::Vector3 p0, geometry_msgs::msg::Vector3 p1);
    double calculateVectorDist(geometry_msgs::msg::Vector3 p);
    std::vector<geometry_msgs::msg::Vector3>
        calculateBezier3Points(geometry_msgs::msg::Vector3 p0, geometry_msgs::msg::Vector3 p1, geometry_msgs::msg::Vector3 p2, geometry_msgs::msg::Vector3 p3, double t);
    std::vector<geometry_msgs::msg::Vector3>
        calculateBezier5Points(geometry_msgs::msg::Vector3 p0, geometry_msgs::msg::Vector3 p1, geometry_msgs::msg::Vector3 p2,
            geometry_msgs::msg::Vector3 p3, geometry_msgs::msg::Vector3 p4, geometry_msgs::msg::Vector3 p5, double t);
    
    rclcpp::Publisher<robot_msgs::msg::PoseArray>::SharedPtr path_points_pub_;

};
}  // namespace robot_path_creater
#endif  // ROBOT_PATH_CREATER__PATH_CREATER_HPP_
