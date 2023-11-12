// Copyright 2023 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef EXTERNAL_MONITOR__EXTERNAL_MONITOR_CORE_HPP_
#define EXTERNAL_MONITOR__EXTERNAL_MONITOR_CORE_HPP_

# include <rclcpp/rclcpp.hpp>

#include <tier4_system_msgs/msg/operation_mode_availability.hpp>


using tier4_system_msgs::msg::OperationModeAvailability;

class ExtenalMonitor : public rclcpp::Node
{
public:
  explicit ExtenalMonitor();

private:
  // Subscriber
  rclcpp::Subscription<OperationModeAvailability>::SharedPtr sub_external_self_monitoring_;
  rclcpp::Subscription<OperationModeAvailability>::SharedPtr sub_external_module_result_;
  rclcpp::Subscription<OperationModeAvailability>::SharedPtr sub_another_external_self_monitoring_;
  rclcpp::Subscription<OperationModeAvailability>::SharedPtr sub_another_external_module_result_;

  // Publisher
  rclcpp::Publisher<OperationModeAvailability>::SharedPtr pub_external_monitoring_;
  rclcpp::Subscription<OperationModeAvailability>::SharedPtr pub_another_external_monitering_;

  void onExternalSelfMonitoring(const OperationModeAvailability::ConstSharedPtr msg);
  void onAnotherExternalSelfMonitoring(const OperationModeAvailability::ConstSharedPtr msg);
  void onExternalModuleResult(const OperationModeAvailability::ConstSharedPtr msg);
  void onAnotherExternalModuleResult(const OperationModeAvailability::ConstSharedPtr msg)
};

#endif  // EXTERNAL_MONITOR__EXTERNAL_MONITOR_CORE_HPP_