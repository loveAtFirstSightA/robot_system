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

#ifndef NAV_TO_POSE__NAV_TO_POSE_HPP_
#define NAV_TO_POSE__NAV_TO_POSE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "nav2_msgs/action/navigate_to_pose.hpp"

namespace nav_to_pose
{
class NavToPose : public rclcpp::Node
{
public:
    NavToPose();
    ~NavToPose();

private:
    void timerCallback()
    {
        if (!nav_to_pose_action_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            return;
        }

        // 停止定时器，避免重复发送任务
        timer_->cancel();

        if (arrived_) {
            RCLCPP_INFO(this->get_logger(), "Navigating to Point 2");
            sendGoal(x2_, y2_, yaw2_);
        } else {
            RCLCPP_INFO(this->get_logger(), "Navigating to Point 1");
            sendGoal(x1_, y1_, yaw1_);
        }

        arrived_ = !arrived_;  // 切换到下一个点
    }
    void sendGoal(double x, double y, double yaw);
    
    using ClientGoalHandle = rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;
    void goal_response_callback(const ClientGoalHandle::SharedPtr & future)
    {
        auto goal_handle = future.get();
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }
    
    void feedback_callback(
        ClientGoalHandle::SharedPtr,
        const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), "Received feedback");
    }
    
    void result_callback(const ClientGoalHandle::WrappedResult & result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Server successfully executed goal");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                return;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                return;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                return;
        }
        // 重新启动定时器，发送下一个任务
        timer_->reset();
    }

    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_to_pose_action_client_;
    rclcpp::TimerBase::SharedPtr timer_;
    double x1_;
    double y1_;
    double yaw1_;
    double x2_;
    double y2_;
    double yaw2_;

    bool arrived_;

};
}  // namespace nav_to_pose
#endif  // NAV_TO_POSE__NAV_TO_POSE_HPP_
