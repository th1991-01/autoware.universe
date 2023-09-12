// Copyright 2022 TIER IV, Inc.
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

#include "motion.hpp"

#include <memory>
#include <unordered_map>

namespace default_ad_api
{

MotionNode::MotionNode(const rclcpp::NodeOptions & options)
: Node("motion", options), vehicle_stop_checker_(this)
{
  stop_check_duration_ = declare_parameter<double>("stop_check_duration");
  require_accept_start_ = declare_parameter<bool>("require_accept_start");
  is_calling_set_pause_ = false;

  const auto adaptor = component_interface_utils::NodeAdaptor(this);
  group_cli_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  adaptor.init_srv(srv_accept_, this, &MotionNode::on_accept);
  adaptor.init_pub(pub_state_);
  adaptor.init_cli(cli_set_pause_, group_cli_);
  adaptor.init_sub(sub_is_paused_, this, &MotionNode::on_is_paused);
  adaptor.init_sub(sub_is_start_requested_, this, &MotionNode::on_is_start_requested);

  rclcpp::Rate rate(10);
  timer_ = rclcpp::create_timer(this, get_clock(), rate.period(), [this]() { on_timer(); });
  state_ = State::Unknown;
}

void MotionNode::update_state()
{
  if (!is_paused_ || !is_start_requested_) {
    RCLCPP_INFO(get_logger(), "update_state(): no update. is_paused_ = %d, is_start_requested_ = %d", is_paused_, is_start_requested_);
    return;
  }

  const auto get_next_state = [this]() {
    RCLCPP_INFO(get_logger(), "get_next_state(): is_paused_ = %d, is_start_requested_ = %d, require_accept_start_ = %d, stop_check_duration_ = %f", is_paused_.value(), is_start_requested_.value(), require_accept_start_, stop_check_duration_);
    if (is_paused_.value()) {
      if (!is_start_requested_.value()) {
        RCLCPP_INFO(get_logger(), "get_next_state(): next state = State::Pause");
        return State::Paused;
      } else {
        RCLCPP_INFO(get_logger(), "get_next_state(): next state = %s", std::string(require_accept_start_ ? "State::Starting" : "State::Resuming"));
        return require_accept_start_ ? State::Starting : State::Resuming;
      }
    } else {
      if (!vehicle_stop_checker_.isVehicleStopped(stop_check_duration_)) {
        RCLCPP_INFO(get_logger(), "get_next_state(): next state = State::Moving");
        return State::Moving;
      } else {
        RCLCPP_INFO(get_logger(), "get_next_state(): next state = %s", std::string(is_start_requested_.value() ? "State::Resumed" : "State::Pausing"));
        return is_start_requested_.value() ? State::Resumed : State::Pausing;
      }
    }
  };
  const auto next_state = get_next_state();

  // Once the state becomes pausing, it must become a state where is_paused is true
  if (state_ == State::Pausing) {
    switch (next_state) {
      case State::Paused:
      case State::Starting:
      case State::Resuming:
        break;
      case State::Moving:
      case State::Pausing:
      case State::Resumed:
      case State::Unknown:
        RCLCPP_INFO(get_logger(), "update_state(): current state = %u, next_state = %u. do nothing.", state_, next_state);
        return;
    }
  }

  // Prevents transition from starting to resuming
  if (state_ == State::Resuming && next_state == State::Starting) {
    RCLCPP_INFO(get_logger(), "update_state(): current state = Resuming, next_state = Starting. do nothing.");
    return;
  }

  change_state(next_state);
}

void MotionNode::change_state(const State state)
{

  static const auto str = std::unordered_map<State, std::string>(
    {{State::Unknown, "State::Unknown"},
     {State::Pausing, "State::Pausing"},
     {State::Paused, "State::Paused"},
     {State::Starting, "State::Starting"},
     {State::Resuming, "State::Resuming"},
     {State::Resumed, "State::Resumed"},
     {State::Moving, "State::Moving"}});

  using MotionState = autoware_ad_api::motion::State::Message;
  static const auto mapping = std::unordered_map<State, MotionState::_state_type>(
    {{State::Unknown, MotionState::UNKNOWN},
     {State::Pausing, MotionState::STOPPED},
     {State::Paused, MotionState::STOPPED},
     {State::Starting, MotionState::STARTING},
     {State::Resuming, MotionState::MOVING},
     {State::Resumed, MotionState::MOVING},
     {State::Moving, MotionState::MOVING}});

  if (mapping.at(state_) != mapping.at(state)) {
    RCLCPP_INFO(get_logger(), "change_state(): UPDATE! current state = %s, next_state = %s.", str.at(state_), str.at(state));
    MotionState msg;
    msg.stamp = now();
    msg.state = mapping.at(state);
    pub_state_->publish(msg);
  }
  state_ = state;
  update_pause(state);
}

void MotionNode::update_pause(const State state)
{
  if (state == State::Pausing) {
    RCLCPP_INFO(get_logger(), "update_pause(): state = Pausing. set pause true");
    return change_pause(true);
  }
  if (state == State::Resuming) {
    RCLCPP_INFO(get_logger(), "update_pause(): state = Resuming. set pause false");
    return change_pause(false);
  }
}

void MotionNode::change_pause(bool pause)
{
  if (!is_calling_set_pause_ && cli_set_pause_->service_is_ready()) {
    const auto req = std::make_shared<control_interface::SetPause::Service::Request>();
    req->pause = pause;
    is_calling_set_pause_ = true;
    cli_set_pause_->async_send_request(req, [this](auto) { is_calling_set_pause_ = false; });
  }
}

void MotionNode::on_timer()
{
  update_state();
}

void MotionNode::on_is_paused(const control_interface::IsPaused::Message::ConstSharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "on_is_paused is called. is_paused_ = %d", msg->data);
  is_paused_ = msg->data;
  update_state();
}

void MotionNode::on_is_start_requested(
  const control_interface::IsStartRequested::Message::ConstSharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "on_is_start_requested is called. is_start_requested_ = %d", msg->data);
  is_start_requested_ = msg->data;
  update_state();
}

void MotionNode::on_accept(
  const autoware_ad_api::motion::AcceptStart::Service::Request::SharedPtr,
  const autoware_ad_api::motion::AcceptStart::Service::Response::SharedPtr res)
{
  if (state_ != State::Starting) {
    using AcceptStartResponse = autoware_ad_api::motion::AcceptStart::Service::Response;
    throw component_interface_utils::ServiceException(
      AcceptStartResponse::ERROR_NOT_STARTING, "The motion state is not starting");
  }
  change_state(State::Resuming);
  res->status.success = true;
}

}  // namespace default_ad_api

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(default_ad_api::MotionNode)
