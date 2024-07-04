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

#ifndef MODEL_PREDICT_CONTROL__MODEL_PREDICT_CONTROL_HPP_
#define MODEL_PREDICT_CONTROL__MODEL_PREDICT_CONTROL_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "algorithm_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "model_predict_control/logger.hpp"
#include "model_predict_control/common.hpp"

namespace model_predict_control
{
class ModelPredictControl : public rclcpp::Node
{
public:
    ModelPredictControl();
    ~ModelPredictControl();

private:
    void initFirstValue();
    void currentPoseCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg);
    void sendVelocity(const double v, const double w);
    void pathSubCallback(const algorithm_msgs::msg::Path::SharedPtr msg);
    
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr current_pose_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_;
    rclcpp::Subscription<algorithm_msgs::msg::Path>::SharedPtr path_sub_;
    double v_;
    double w_;
    bool is_path_received_{false};
    algorithm_msgs::msg::Path path_;


};
}  // namespace model_predict_control
#endif  // MODEL_PREDICT_CONTROL__MODEL_PREDICT_CONTROL_HPP_
