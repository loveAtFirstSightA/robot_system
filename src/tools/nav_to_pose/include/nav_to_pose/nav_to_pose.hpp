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

#include <memory>
#include "chrono"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "tf2/utils.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

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
        task_count_ = task_count_ + 1;
        if (task_count_ > 2000) {
            return;
        }
        if (!nav_to_pose_action_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            return;
        }

        // 停止定时器，避免重复发送任务
        timer_->cancel();

        if (arrived_) {
            RCLCPP_INFO(this->get_logger(), "Task %d: Navigating to Point 2 [x %.4f, y %.4f, yaw %.4f]", task_count_, x2_, y2_, yaw2_);
            sendGoal(x2_, y2_, yaw2_);
        } else {
            RCLCPP_INFO(this->get_logger(), "Task %d: Navigating to Point 1 [x %.4f, y %.4f, yaw %.4f]", task_count_, x1_, y1_, yaw1_);
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
        RCLCPP_DEBUG(this->get_logger(), "Received feedback");
    }
    
    void result_callback(const ClientGoalHandle::WrappedResult & result)
    {
        double x, y, yaw;
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED: 
            {
                rclcpp::sleep_for(std::chrono::seconds(1));
                RCLCPP_INFO(this->get_logger(), "Server successfully executed goal");
                while (!getCurrentPose(x, y, yaw)) {
                }
                RCLCPP_INFO(this->get_logger(), "Arrived current pose: [x %.4f, y %.4f, yaw %.4f]", x, y, yaw);
            } break;
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

    bool getCurrentPose(double & x, double & y, double & yaw)
    {
        std::string target_link = "map";
        std::string source_link = "base_footprint";
        geometry_msgs::msg::TransformStamped tf;
        try {
            tf = tf_buffer_->lookupTransform(target_link, source_link, tf2::TimePointZero);
            x = tf.transform.translation.x;
            y = tf.transform.translation.y;
            yaw = tf2::getYaw(tf.transform.rotation);
            return true;
        } catch(const std::exception& e) {
            std::cerr << e.what() << '\n';
        }
        return false;
    }

    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_to_pose_action_client_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    double x1_;
    double y1_;
    double yaw1_;
    double x2_;
    double y2_;
    double yaw2_;

    bool arrived_;

    int task_count_{0};

};
}  // namespace nav_to_pose
#endif  // NAV_TO_POSE__NAV_TO_POSE_HPP_
