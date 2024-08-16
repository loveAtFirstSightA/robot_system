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

#ifndef SWITCH_MAP_UNIT_TEST__SWITCH_MAP_UNIT_TEST_HPP_
#define SWITCH_MAP_UNIT_TEST__SWITCH_MAP_UNIT_TEST_HPP_

#include "string"
#include "rclcpp/rclcpp.hpp"
#include "fcbox_msgs/srv/amcl_status_control.hpp"
#include "fcbox_msgs/srv/nav_status_control.hpp"
#include "spdlog/spdlog.h"
#include "fcbox_msgs/srv/change_state.hpp"

namespace switch_map_unit_test
{
class SwitchMapUnitTest : public rclcpp::Node
{
public:
    SwitchMapUnitTest();
    ~SwitchMapUnitTest();

private:
    void timerCallback();
    using AmclServiceResponseFuture = rclcpp::Client<fcbox_msgs::srv::AmclStatusControl>::SharedFuture;
    using NavServiceResponseFuture = rclcpp::Client<fcbox_msgs::srv::NavStatusControl>::SharedFuture;
    void enableAmclClientResponseCallback(AmclServiceResponseFuture future);
    void enableNavClientResponseCallback(NavServiceResponseFuture future);
    void disableAmclClientResponseCallback(AmclServiceResponseFuture future);
    void disableNavClientResponseCallback(NavServiceResponseFuture future);
    bool isAmclServerAvailable();
    bool isNavServerAvailable();

    rclcpp::Client<fcbox_msgs::srv::AmclStatusControl>::SharedPtr amcl_cli_;
    rclcpp::Client<fcbox_msgs::srv::NavStatusControl>::SharedPtr nav_cli_;
    rclcpp::TimerBase::SharedPtr timer_;
    unsigned int unit_test_count_;
    std::string a_map_path_;
    std::string b_map_path_;

private:
    rclcpp::Service<fcbox_msgs::srv::ChangeState>::SharedPtr change_state_ser_;

    
};
}  // namespace switch_map_unit_test
#endif  // SWITCH_MAP_UNIT_TEST__SWITCH_MAP_UNIT_TEST_HPP_
