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

#include "external_monitor/external_monitor_core.hpp"

using tier4_system_msgs::msg::OperationModeAvailability;

ExternalMonitor::ExternalMonitor()
: Node("external_monitor")
{
  using std::placeholders::_1;

  sub_external_self_monitoring_ = create_subscription<OperationModeAvailability>(
    "~/input/external/self_monitoring", rclcpp::QoS{1}, std::bind(&ExternalMonitor::onExternalSelfMonitoring, this, _1));
  sub_external_module_result_ = create_subscription<OperationModeAvailability>(
    "~/input/external/module_result", rclcpp::QoS{1}, std::bind(&ExternalMonitor::onExternalModuleResult, this, _1));
  sub_another_external_self_monitoring_ = create_subscription<OperationModeAvailability>(
    "~/input/another_external/self_monitoring", rclcpp::QoS{1}, std::bind(&ExternalMonitor::onExternalSelfMonitoring, this, _1));
  sub_another_external_module_result_ = create_subscription<OperationModeAvailability>(
    "~/input/another_external/module_result", rclcpp::QoS{1}, std::bind(&ExternalMonitor::onAnotherExternalModuleResult, this, _1));


  pub_external_monitoring_ = create_publisher<OperationModeAvailability>("~/output/external_monitoring", rclcpp::QoS{1});
  pub_another_external_monitering_ = create_publisher<OperationModeAvailability>("~/output/another_external_monitoring", rclcpp::QoS{1});
}

// For now, the results of Self_monitoring are also being used as the results of External_Monitoring.
void ExternalMonitor::onExternalSelfMonitoring(const OperationModeAvailability::ConstSharedPtr msg) {
  pub_external_monitoring_->publish(*msg);
}

// For now, the results of Self_monitoring are also being used as the results of External_Monitoring.
void ExternalMonitor::onAnotherExternalSelfMonitoring(const OperationModeAvailability::ConstSharedPtr msg) {
  pub_another_external_monitering_->publish(*msg);
}

// Ideally processes the topic from the external monitoring module.
// The message type is provisionally set to "OperationModeAvailability" for now.
void ExternalMonitor::onExternalModuleResult([[maybe_unused]] const OperationModeAvailability::ConstSharedPtr msg) {
  // Implementation goes here
}

// Ideally processes the topic from the external monitoring module.
// The message type is provisionally set to "OperationModeAvailability" for now.
void ExternalMonitor::onAnotherExternalModuleResult([[maybe_unused]] const OperationModeAvailability::ConstSharedPtr msg) {
  // Implementation goes here
}