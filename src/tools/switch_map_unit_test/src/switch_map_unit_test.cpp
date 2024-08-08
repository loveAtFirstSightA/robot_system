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
     std::cout << getCurrentTime() << "[INFO] " << "Start execute unit test of switch map" << std::endl;
     amcl_cli_ = this->create_client<fcbox_msgs::srv::AmclStatusControl>("amcl_status_control");
     std::cout << getCurrentTime() << "[INFO] " << "Create client to request amcl_status_control" << std::endl;
     nav_cli_ = this->create_client<fcbox_msgs::srv::NavStatusControl>("nav_status_control");
     std::cout << getCurrentTime() << "[INFO] " << "Create client to request nav_status_control" << std::endl;
     timer_ = this->create_wall_timer(std::chrono::seconds(5), std::bind(&SwitchMapUnitTest::timerCallback, this));

     unit_test_count_ = 0;
     a_map_path_ = "/home/fcbox/.fcbox/map/1717573160772/MAP_-1A楼地图_1717573160.yaml";
     b_map_path_ = "/home/fcbox/.fcbox/map/1721110225254/MAP_20floor_1721110405.yaml";
}

SwitchMapUnitTest::~SwitchMapUnitTest() {}

void SwitchMapUnitTest::timerCallback()
{
     std::cout << std::endl;
     std::cout << getCurrentTime() << "[INFO] " << "Unit test count: " << ++unit_test_count_ << std::endl;
     std::cout << getCurrentTime() << "[INFO] " << "Timer event - switch map" << std::endl;

     // disable nav
     if (!isNavServerAvailable()) {
          std::cout << getCurrentTime() << "[ERROR] " << "NAV_INTERFACE server not available!" << std::endl;
          std::cout << getCurrentTime() << "[INFO] " << "Cancel timer" << std::endl;
          timer_->cancel();
          return;
     } else {
          std::cout << getCurrentTime() << "[INFO] " << "NAV_INTERFACE server is available" << std::endl;
     }

     auto request = std::make_shared<fcbox_msgs::srv::NavStatusControl::Request>();
     request->target_status = false;
     request->map_path = "";
     std::cout << getCurrentTime() << "[INFO] " << "Sending request to " << (request->target_status ? "enable" : "disable") << " navigation"  << std::endl;
     std::cout << getCurrentTime() << "[INFO] " << "    - Target_status: " << request->target_status << std::endl;
     std::cout << getCurrentTime() << "[INFO] " << "    - Map path: " << request->map_path.c_str() << std::endl;
     nav_cli_->async_send_request(
          request, std::bind(&SwitchMapUnitTest::disableNavClientResponseCallback, this, std::placeholders::_1));

     if (!isAmclServerAvailable()) {
          std::cout << getCurrentTime() << "[ERROR] " << "AMCL server not available!" << std::endl;
          std::cout << getCurrentTime() << "[INFO] " << "Cancel timer" << std::endl;
          timer_->cancel();
          return;
     } else {
          std::cout << getCurrentTime() << "[INFO] " << "AMCL server is available" << std::endl;
     }

}

void SwitchMapUnitTest::enableAmclClientResponseCallback(AmclServiceResponseFuture future)
{
     auto response = future.get();
     std::cout << getCurrentTime() << "[INFO] " << "Received response" << std::endl;
     std::cout << getCurrentTime() << "[INFO] " << "    - Result: " << response->result << std::endl;
     std::cout << getCurrentTime() << "[INFO] " << "    - Msg: " << response->msg << std::endl;

     if (!isNavServerAvailable()) {
          std::cout << getCurrentTime() << "[ERROR] " << "NAV_INTERFACE server not available!" << std::endl;
          std::cout << getCurrentTime() << "[INFO] " << "Cancel timer" << std::endl;
          timer_->cancel();
          return;
     } else {
          std::cout << getCurrentTime() << "[INFO] " << "NAV_INTERFACE server is available" << std::endl;
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
     std::cout << getCurrentTime() << "[INFO] " << "Sending request to " << (request->target_status ? "enable" : "disable") << " navigation"  << std::endl;
     std::cout << getCurrentTime() << "[INFO] " << "    - Target_status: " << request->target_status << std::endl;
     std::cout << getCurrentTime() << "[INFO] " << "    - Map path: " << request->map_path.c_str() << std::endl;
     nav_cli_->async_send_request(
          request, std::bind(&SwitchMapUnitTest::enableNavClientResponseCallback, this, std::placeholders::_1));
}

void SwitchMapUnitTest::enableNavClientResponseCallback(NavServiceResponseFuture future)
{
     auto response = future.get();
     std::cout << getCurrentTime() << "[INFO] " << "Received response" << std::endl;
     std::cout << getCurrentTime() << "[INFO] " << "    - Result: " << response->result << std::endl;
     std::cout << getCurrentTime() << "[INFO] " << "    - Msg: " << response->msg << std::endl;

}

void SwitchMapUnitTest::disableAmclClientResponseCallback(AmclServiceResponseFuture future)
{
     auto response = future.get();
     std::cout << getCurrentTime() << "[INFO] " << "Received response" << std::endl;
     std::cout << getCurrentTime() << "[INFO] " << "    - Result: " << response->result << std::endl;
     std::cout << getCurrentTime() << "[INFO] " << "    - Msg: " << response->msg << std::endl;

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

     std::cout << getCurrentTime() << "[INFO] " << "Sending request to " << (request->target_status ? "enable" : "disable") << " amcl" << std::endl;
     std::cout << getCurrentTime() << "[INFO] " << "    - Frame ID: " << request->init_pose.header.frame_id << std::endl;
     std::cout << getCurrentTime() << "[INFO] " << "    - Timestamp: " << request->init_pose.header.stamp.sec << "." << request->init_pose.header.stamp.nanosec << std::endl;
     std::cout << getCurrentTime() << "[INFO] " << "    - Position: (" 
               << request->init_pose.pose.position.x << ", " 
               << request->init_pose.pose.position.y << ", " 
               << request->init_pose.pose.position.z << ")" << std::endl;
     std::cout << getCurrentTime() << "[INFO] " << "    - Orientation: ("
               << request->init_pose.pose.orientation.w << ", "
               << request->init_pose.pose.orientation.x << ", "
               << request->init_pose.pose.orientation.y << ", "
               << request->init_pose.pose.orientation.z << ")" << std::endl;

     amcl_cli_->async_send_request(
          request, std::bind(&SwitchMapUnitTest::enableAmclClientResponseCallback, this, std::placeholders::_1));

}

void SwitchMapUnitTest::disableNavClientResponseCallback(NavServiceResponseFuture future)
{
     auto response = future.get();
     std::cout << getCurrentTime() << "[INFO] " << "Received response" << std::endl;
     std::cout << getCurrentTime() << "[INFO] " << "    - Result: " << response->result << std::endl;
     std::cout << getCurrentTime() << "[INFO] " << "    - Msg: " << response->msg << std::endl;

     if (!isAmclServerAvailable()) {
          std::cout << getCurrentTime() << "[ERROR] " << "AMCL server not available!" << std::endl;
          std::cout << getCurrentTime() << "[INFO] " << "Cancel timer" << std::endl;
          timer_->cancel();
          return;
     } else {
          std::cout << getCurrentTime() << "[INFO] " << "AMCL server is available" << std::endl;
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

     std::cout << getCurrentTime() << "[INFO] " << "Sending request to " << (request->target_status ? "enable" : "disable") << " amcl" << std::endl;
     std::cout << getCurrentTime() << "[INFO] " << "    - Frame ID: " << request->init_pose.header.frame_id << std::endl;
     std::cout << getCurrentTime() << "[INFO] " << "    - Timestamp: " << request->init_pose.header.stamp.sec << "." << request->init_pose.header.stamp.nanosec << std::endl;
     std::cout << getCurrentTime() << "[INFO] " << "    - Position: (" 
               << request->init_pose.pose.position.x << ", " 
               << request->init_pose.pose.position.y << ", " 
               << request->init_pose.pose.position.z << ")" << std::endl;
     std::cout << getCurrentTime() << "[INFO] " << "    - Orientation: ("
               << request->init_pose.pose.orientation.w << ", "
               << request->init_pose.pose.orientation.x << ", "
               << request->init_pose.pose.orientation.y << ", "
               << request->init_pose.pose.orientation.z << ")" << std::endl;

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
               std::cout << getCurrentTime() << "[INFO] " << "AMCL server not available, retry " << retry_count << "/5..." << std::endl;
          }
     }
     std::cout << getCurrentTime() << "[ERROR] " << "AMCL server not available after 5 attempts." << std::endl;
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
               std::cout << getCurrentTime() << "[INFO] " << "NAV_INTERFACE server not available, retry " << retry_count << "/5..." << std::endl;
          }
     }
     std::cout << getCurrentTime() << "[ERROR] " << "NAV_INTERFACE server not available after 5 attempts." << std::endl;
     return false;
}


}  // namespace switch_map_unit_test
