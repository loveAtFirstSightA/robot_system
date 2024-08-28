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

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "spdlog/spdlog.h"
#include "fcbox_msgs/srv/change_state.hpp"

namespace switch_map_unit_test
{
class ChangeState : public rclcpp::Node
{
public:
    ChangeState() : Node("change_state")
    {
        change_state_client_ = this->create_client<fcbox_msgs::srv::ChangeState>("/ChangeRobotState");
        timer_ = this->create_wall_timer(
            std::chrono::seconds(10),
            std::bind(&ChangeState::timerCallabck, this));
        spdlog::info("Create client to change_state");
        spdlog::info("Create timer of 10s");
        count_ = 0;
        map_id_1 = 1711768927165;
        map_id_2 = 1709877929284;
    }

private:
    void timerCallabck()
    {
        std::cout << std::endl;
        spdlog::info("Timer event: {}", ++count_);
        auto request = std::make_shared<fcbox_msgs::srv::ChangeState::Request>();
        request->target_state = request->NAVIGATION;
        request->pose.position.x = 0.0f;
        request->pose.position.y = 0.0f;
        request->pose.position.z = 0.0f;
        request->pose.orientation.w = 1.0f;
        request->pose.orientation.x = 0.0f;
        request->pose.orientation.y = 0.0f;
        request->pose.orientation.z = 0.0f;
        if (count_ % 2 == 0) {
            request->map_id = map_id_1;
        } else {
            request->map_id = map_id_2;
        }
        spdlog::info(" - Sending request: state {}, pose [x {:.4f}, y {:.4f}, z {:.4f}, w {:.4f}, x {:.4f} y {:.4f}, z {:.4f}], map_id: {}",
            request->target_state, request->pose.position.x, request->pose.position.y, request->pose.position.z, request->pose.orientation.w, 
            request->pose.orientation.x, request->pose.orientation.y, request->pose.orientation.z, request->map_id);
        change_state_client_->async_send_request(
            request, std::bind(&ChangeState::changeStateClientCallback, this, std::placeholders::_1));
    }
    using ServiceResponseFuture = rclcpp::Client<fcbox_msgs::srv::ChangeState>::SharedFuture;
    void changeStateClientCallback(ServiceResponseFuture future)
    {
        try {
            auto response = future.get();
            if (response) {
                spdlog::info(" - Received response: result {}, msg {}", response->result, response->message);
            } else {
                spdlog::warn(" - Received null response");
            }
        } catch (const std::exception &e) {
            spdlog::error(" - Exception caught while getting response: {}", e.what());
        }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Client<fcbox_msgs::srv::ChangeState>::SharedPtr change_state_client_;
    int count_;
    uint64_t map_id_1;
    uint64_t map_id_2;
};

} // namespace switch_map_unit_test

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<switch_map_unit_test::ChangeState>());
    rclcpp::shutdown();
    return 0;
}

