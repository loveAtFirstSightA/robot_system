#include "task_nav_with_charge/task_nav_with_charge.hpp"

namespace task_nav_with_charge 
{

TaskNavWithCharge::TaskNavWithCharge() : Node("task_nav_with_charge") 
{
    // Initialize waypoint list (adjust according to actual scenario)
    waypoints_ = {
        std::make_tuple(-0.1709, -0.0543, 2.4464),
        std::make_tuple(-4.65403, -33.16840, 0.08517),
        std::make_tuple(-0.1709, -0.0543, 2.4464),
        std::make_tuple(-3.81248, -36.06179, -3.06984)
    };
    timer_ = this->create_wall_timer(std::chrono::seconds(3), std::bind(&TaskNavWithCharge::TimerCallback, this));
    // Subscribe to battery percentage topic
    battery_subscription_ = this->create_subscription<std_msgs::msg::Float32>("/battery_percentage", 10,
        std::bind(&TaskNavWithCharge::BatteryCallback, this, std::placeholders::_1));
    // Create publisher
    app_request_publisher_ = this->create_publisher<std_msgs::msg::String>("/app_transparent_request", 10);
    nav_cli_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");
}

TaskNavWithCharge::~TaskNavWithCharge() 
{
    CancelAllGoals();
}

void TaskNavWithCharge::TimerCallback() 
{
    static int current_waypoint_index = 0;  // 用于跟踪当前任务点索引

    if (!is_nav_task_active_) {
        // 获取当前任务点坐标
        auto [x, y, yaw] = waypoints_[current_waypoint_index];
        ExecuteNavTask(x, y, yaw);
        spdlog::info("Executing navigation task to waypoint {}: ({:.2f}, {:.2f}, {:.2f})", current_waypoint_index + 1, x, y, yaw);
        // 更新索引以指向下一个任务点
        current_waypoint_index = (current_waypoint_index + 1) % waypoints_.size(); // 循环回到第一个点
        is_nav_task_active_ = true;  // 设置为正在执行导航任务
    }

    // if (battery_percentage_ < kStartChargeThreshold) {
    //     if (!is_charging_) {
    //         CancelAllGoals();
    //         StartCharging();
    //         is_charging_ = true;
    //         spdlog::info("Battery low {:.2f}, canceling all goals and starting recharge.", battery_percentage_);
    //     }
    // } else if (battery_percentage_ > kStopChargeThreshold) {
    //     if (is_charging_) {
    //         StopCharging();
    //         is_charging_ = false;
    //     }
    //     if (!is_nav_task_active_) {
    //         // 获取当前任务点坐标
    //         auto [x, y, yaw] = waypoints_[current_waypoint_index];
    //         ExecuteNavTask(x, y, yaw);
    //         spdlog::info("Executing navigation task to waypoint {}: ({:.2f}, {:.2f}, {:.2f})", current_waypoint_index + 1, x, y, yaw);
    //         // 更新索引以指向下一个任务点
    //         current_waypoint_index = (current_waypoint_index + 1) % waypoints_.size(); // 循环回到第一个点
    //         is_nav_task_active_ = true;  // 设置为正在执行导航任务
    //     }
    // }
}

void TaskNavWithCharge::BatteryCallback(const std_msgs::msg::Float32::SharedPtr msg) 
{
    battery_percentage_ = msg->data;
}

void TaskNavWithCharge::StartCharging() 
{
    static int count;
    spdlog::info("Number {} charge task", count ++);
    // Start charging
    std::string command = "ros2 param set /auto_recharge_node current_state 1";
    system(command.c_str());

    std_msgs::msg::String msg;
    msg.data = "{\"cmdFunction\": 2564, \"cmdTime\": 123, \"msgId\": 111749980}";
    app_request_publisher_->publish(msg);
    spdlog::info("Execute the charging task");
}

void TaskNavWithCharge::StopCharging() 
{
    std_msgs::msg::String msg;
    msg.data = "{\"cmdFunction\": 2565, \"cmdTime\": 123, \"msgId\": 111749980}";
    app_request_publisher_->publish(msg);
    spdlog::info("Execute the stop charging task");
}

void TaskNavWithCharge::ExecuteNavTask(double x, double y, double yaw) 
{
    static int count;
    spdlog::info("Number {} nav task", count ++);
    if (!IsServerAvailable()) {
        return;
    }

    auto goal = nav2_msgs::action::NavigateToPose::Goal();
    goal.behavior_tree = "";
    goal.pose.header.frame_id = "map";
    goal.pose.header.stamp = rclcpp::Clock().now();

    goal.pose.pose.position.x = x;
    goal.pose.pose.position.y = y;
    goal.pose.pose.position.z = 0.0f;

    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);  // Set roll and pitch to 0, use yaw to generate quaternion

    goal.pose.pose.orientation.x = q.x();
    goal.pose.pose.orientation.y = q.y();
    goal.pose.pose.orientation.z = q.z();
    goal.pose.pose.orientation.w = q.w();
    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&TaskNavWithCharge::GoalResponseCallback, this, std::placeholders::_1);
    send_goal_options.feedback_callback = std::bind(&TaskNavWithCharge::FeedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback = std::bind(&TaskNavWithCharge::ResultCallback, this, std::placeholders::_1);
    spdlog::info("Sending goal to (x: {:.2f}, y: {:.2f}, yaw: {:.2f})", x, y, yaw);
    nav_cli_->async_send_goal(goal, send_goal_options);
}

bool TaskNavWithCharge::IsServerAvailable() 
{
    if (!nav_cli_->wait_for_action_server(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            spdlog::error("Interrupted while waiting for the action server.");
            return false;
        }
        spdlog::info("Action server not available, waiting...");
    }
    return true;
}

void TaskNavWithCharge::CancelAllGoals() 
{
    spdlog::info("Canceling all navigation goals.");
    nav_cli_->async_cancel_all_goals();
}

void TaskNavWithCharge::GoalResponseCallback(const ClientGoalHandle::SharedPtr & future) 
{
    auto goal_handle = future.get();
    if (!goal_handle) {
        spdlog::error("Goal rejected by server");
    } else {
        spdlog::info("Goal accepted by server");
    }
}

void TaskNavWithCharge::FeedbackCallback(ClientGoalHandle::SharedPtr,
    const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback) 
{
    static int count;
    if (count ++ > 500) {
        count = 0;
        spdlog::info("Bot pose [x {:.4f}, y {:.4f}, yaw {:.4f}]",
            feedback->current_pose.pose.position.x, feedback->current_pose.pose.position.y, tf2::getYaw(feedback->current_pose.pose.orientation));
    }
}

void TaskNavWithCharge::ResultCallback(const ClientGoalHandle::WrappedResult & result) 
{
    is_nav_task_active_ = false;
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
        spdlog::info("Goal successfully reached.");
        break;
        case rclcpp_action::ResultCode::ABORTED:
        spdlog::error("Goal was aborted.");
        break;
        case rclcpp_action::ResultCode::CANCELED:
        spdlog::error("Goal was canceled.");
        break;
        default:
        spdlog::error("Unknown result code.");
        break;
    }
}

}  // namespace task_nav_with_charge