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

#include "switch_map_unit_test/switch_map_unit_test.hpp"

namespace switch_map_unit_test
{

SwitchMapUnitTest::SwitchMapUnitTest() : Node("switch_map_unit_test")
{
     spdlog::info("Start execute unit test of switch map");
     amcl_cli_ = this->create_client<fcbox_msgs::srv::AmclStatusControl>("amcl_status_control");
     spdlog::info("Create client to request amcl_status_control");
     nav_cli_ = this->create_client<fcbox_msgs::srv::NavStatusControl>("nav_status_control");
     spdlog::info("Create client to request nav_status_control");
     timer_ = this->create_wall_timer(std::chrono::seconds(3), std::bind(&SwitchMapUnitTest::timerCallback, this));

     unit_test_count_ = 0;
     a_map_path_ = "/home/lio/robot_system/maps/factory.yaml";
     b_map_path_ = "/home/lio/robot_system/maps/factory.yaml";
}

SwitchMapUnitTest::~SwitchMapUnitTest() {}

void SwitchMapUnitTest::timerCallback()
{
     std::cout << std::endl;
     spdlog::info("Unit test count: {}", ++unit_test_count_);
     spdlog::info("Timer event - switch map");

     // disable nav
     if (!isNavServerAvailable()) {
          spdlog::error("NAV_INTERFACE server not available!");
          timer_->cancel();
          spdlog::info("Cancel timer");
          return;
     } else {
          spdlog::info("NAV_INTERFACE server is available");
     }

     auto request = std::make_shared<fcbox_msgs::srv::NavStatusControl::Request>();
     request->target_status = false;
     request->map_path = "";
     spdlog::info("Sending request to {}", request->target_status ? "enable" : "disable");
     spdlog::info("    - Target_status: {}", request->target_status);
     spdlog::info("    - Map path: {}", request->map_path);
     nav_cli_->async_send_request(
          request, std::bind(&SwitchMapUnitTest::disableNavClientResponseCallback, this, std::placeholders::_1));
}

void SwitchMapUnitTest::enableAmclClientResponseCallback(AmclServiceResponseFuture future)
{
     auto response = future.get();
     spdlog::info("Received response");
     spdlog::info("    - Result: {}", response->result);
     spdlog::info("    - Msg: {}", response->msg);

     if (!isNavServerAvailable()) {
          spdlog::error("NAV_INTERFACE server not available!");
          timer_->cancel();
          spdlog::info("Cancel timer");
          return;
     } else {
          spdlog::info("NAV_INTERFACE server is available");
     }

     // enable nav
     auto request = std::make_shared<fcbox_msgs::srv::NavStatusControl::Request>();
     request->target_status = true;
     // TODO
     if (unit_test_count_ % 2 == 0) {
          request->map_path = a_map_path_;
     } else {
          request->map_path = b_map_path_;
     }
     spdlog::info("Sending request to {}", request->target_status ? "enable" : "disable");
     spdlog::info("    - Target_status: {}", request->target_status);
     spdlog::info("    - Map path: {}", request->map_path.c_str());
     nav_cli_->async_send_request(
          request, std::bind(&SwitchMapUnitTest::enableNavClientResponseCallback, this, std::placeholders::_1));
}

void SwitchMapUnitTest::enableNavClientResponseCallback(NavServiceResponseFuture future)
{
    auto response = future.get();
    spdlog::info("Received response");
    spdlog::info("    - Result: {}", response->result);
    spdlog::info("    - Msg: {}", response->msg);
}

void SwitchMapUnitTest::disableAmclClientResponseCallback(AmclServiceResponseFuture future)
{
     auto response = future.get();
     spdlog::info("Received response");
     spdlog::info("    - Result: {}", response->result);
     spdlog::info("    - Msg: {}", response->msg);

     if (!isAmclServerAvailable()) {
          spdlog::error("AMCL server not available!");
          timer_->cancel();
          spdlog::info("Cancel timer");
          return;
     } else {
          spdlog::info("AMCL server is available");
     }

     // enable amcl
     auto request = std::make_shared<fcbox_msgs::srv::AmclStatusControl::Request>();
     request->target_status = true;
     request->init_pose.header.frame_id = "map";
     request->init_pose.header.stamp = rclcpp::Clock().now();
     request->init_pose.pose.position.x = 0.0f;
     request->init_pose.pose.position.y = 0.0f;
     request->init_pose.pose.position.z = 0.0f;
     request->init_pose.pose.orientation.w = 1.0f;
     request->init_pose.pose.orientation.x = 0.0f;
     request->init_pose.pose.orientation.y = 0.0f;
     request->init_pose.pose.orientation.z = 0.0f;

     spdlog::info("Sending request to {}", request->target_status ? "enable" : "disable");
     spdlog::info("    - Frame ID: {}", request->init_pose.header.frame_id);
     spdlog::info("    - Timestamp: {}.{}", request->init_pose.header.stamp.sec, request->init_pose.header.stamp.nanosec);
     spdlog::info("    - Position: ({}, {}, {})", request->init_pose.pose.position.x, request->init_pose.pose.position.y, request->init_pose.pose.position.z);
     spdlog::info("    - Orientation: ({}, {}, {}, {})", request->init_pose.pose.orientation.w, request->init_pose.pose.orientation.x, request->init_pose.pose.orientation.y, request->init_pose.pose.orientation.z);

     amcl_cli_->async_send_request(
          request, std::bind(&SwitchMapUnitTest::enableAmclClientResponseCallback, this, std::placeholders::_1));
}

void SwitchMapUnitTest::disableNavClientResponseCallback(NavServiceResponseFuture future)
{
     auto response = future.get();
     spdlog::info("Received response");
     spdlog::info("    - Result: {}", response->result);
     spdlog::info("    - Msg: {}", response->msg);

     if (!isAmclServerAvailable()) {
          spdlog::error("AMCL server not available!");
          timer_->cancel();
          spdlog::info("Cancel timer");
          return;
     } else {
          spdlog::info("AMCL server is available");
     }

     // disable amcl
     auto request = std::make_shared<fcbox_msgs::srv::AmclStatusControl::Request>();
     request->target_status = false;
     request->init_pose.header.frame_id = "map";
     request->init_pose.header.stamp = rclcpp::Clock().now();
     request->init_pose.pose.position.x = 0.0f;
     request->init_pose.pose.position.y = 0.0f;
     request->init_pose.pose.position.z = 0.0f;
     request->init_pose.pose.orientation.w = 1.0f;
     request->init_pose.pose.orientation.x = 0.0f;
     request->init_pose.pose.orientation.y = 0.0f;
     request->init_pose.pose.orientation.z = 0.0f;

     spdlog::info("Sending request to {}", request->target_status ? "enable" : "disable");
     spdlog::info("    - Frame ID: {}", request->init_pose.header.frame_id);
     spdlog::info("    - Timestamp: {}.{}", request->init_pose.header.stamp.sec, request->init_pose.header.stamp.nanosec);
     spdlog::info("    - Position: ({}, {}, {})", request->init_pose.pose.position.x, request->init_pose.pose.position.y, request->init_pose.pose.position.z);
     spdlog::info("    - Orientation: ({}, {}, {}, {})", request->init_pose.pose.orientation.w, request->init_pose.pose.orientation.x, request->init_pose.pose.orientation.y, request->init_pose.pose.orientation.z);

     amcl_cli_->async_send_request(
          request, std::bind(&SwitchMapUnitTest::disableAmclClientResponseCallback, this, std::placeholders::_1));
}

bool SwitchMapUnitTest::isAmclServerAvailable()
{
     int retry_count = 0;
     while (retry_count < 5) {
          if (amcl_cli_->wait_for_service(std::chrono::seconds(1))) {
               return true; // Server is available
          } else {
               retry_count++;
               spdlog::info("AMCL server not available, retry {}/5...", retry_count);
          }
     }
     spdlog::error("AMCL server not available after 5 attempts.");
     return false;
}

bool SwitchMapUnitTest::isNavServerAvailable()
{
     int retry_count = 0;
     while (retry_count < 5) {
          if (nav_cli_->wait_for_service(std::chrono::seconds(1))) {
               return true; // Server is available
          } else {
               retry_count++;
               spdlog::info("NAV_INTERFACE server not available, retry {}/5...", retry_count);
          }
     }
     spdlog::error("NAV_INTERFACE server not available after 5 attempts.");
     return false;
}

}  // namespace switch_map_unit_test
