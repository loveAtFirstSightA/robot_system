#ifndef TASK_NAV_WITH_CHARGE_TASK_NAV_WITH_CHARGE_HPP_
#define TASK_NAV_WITH_CHARGE_TASK_NAV_WITH_CHARGE_HPP_

#include <memory>
#include <chrono>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "spdlog/spdlog.h"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/utils.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace task_nav_with_charge 
{
class TaskNavWithCharge : public rclcpp::Node 
{
public:
    TaskNavWithCharge();
    ~TaskNavWithCharge();

private:
    void BatteryCallback(const std_msgs::msg::Float32::SharedPtr msg);
    void StartCharging();
    void StopCharging();
    void ExecuteNavTask(double x, double y, double yaw);
    bool IsServerAvailable();
    void CancelAllGoals();
    void TimerCallback();

    // Vector of waypoints
    std::vector<std::tuple<double, double, double>> waypoints_;

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr battery_subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr app_request_publisher_;

    float battery_percentage_;
    const float kStartChargeThreshold = 20.0;
    const float kStopChargeThreshold = 80.0;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_cli_;

    bool is_charging_ = false;
    bool is_nav_task_active_ = false;

    using ClientGoalHandle = rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;
    void GoalResponseCallback(const ClientGoalHandle::SharedPtr & future);
    void FeedbackCallback(ClientGoalHandle::SharedPtr, const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback);
    void ResultCallback(const ClientGoalHandle::WrappedResult & result);

};
}  // namespace task_nav_with_charge

#endif  // TASK_NAV_WITH_CHARGE_TASK_NAV_WITH_CHARGE_HPP_