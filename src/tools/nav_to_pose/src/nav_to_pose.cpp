/*
 Copyright 2024 Google LLC

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

#include "nav_to_pose/nav_to_pose.hpp"

namespace nav_to_pose
{
NavToPose::NavToPose() : Node("nav_to_pose"),
x1_(9.7415f), y1_(-4.8958f), yaw1_(0.7173f),  // 第一个目标点
x2_(-1.1275f), y2_(3.4820f), yaw2_(-0.5457f)   // 第二个目标点
{
     timer_ = this->create_wall_timer(
          std::chrono::seconds(3),
          std::bind(&NavToPose::timerCallback, this));
     nav_to_pose_action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
          this, "/navigate_to_pose");
     tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
     tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

NavToPose::~NavToPose() 
{
     nav_to_pose_action_client_->async_cancel_all_goals();
}

void NavToPose::sendGoal(double x, double y, double yaw)
{
     auto goal = nav2_msgs::action::NavigateToPose::Goal();
     goal.behavior_tree = "";
     goal.pose.header.frame_id = "map";
     goal.pose.header.stamp = rclcpp::Clock().now();

     goal.pose.pose.position.x = x;
     goal.pose.pose.position.y = y;
     goal.pose.pose.position.z = 0.0f;

     tf2::Quaternion q;
     q.setRPY(0, 0, yaw);  // 将roll和pitch设为0，使用yaw生成四元数

     goal.pose.pose.orientation.x = q.x();
     goal.pose.pose.orientation.y = q.y();
     goal.pose.pose.orientation.z = q.z();
     goal.pose.pose.orientation.w = q.w();
     auto send_goal_options =
          rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
     send_goal_options.goal_response_callback =
          std::bind(&NavToPose::goal_response_callback, this, std::placeholders::_1);
     send_goal_options.feedback_callback =
          std::bind(&NavToPose::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
     send_goal_options.result_callback =
          std::bind(&NavToPose::result_callback, this, std::placeholders::_1);
     RCLCPP_INFO(this->get_logger(), "Sending goal");
     nav_to_pose_action_client_->async_send_goal(goal, send_goal_options);
}

}  // namespace nav_to_pose
