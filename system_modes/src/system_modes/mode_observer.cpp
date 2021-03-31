// Copyright (c) 2018 - for information on the respective copyright owner
// see the NOTICE file and/or the repository https://github.com/microros/system_modes
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
#include "system_modes/mode_observer.hpp"

#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition_event.hpp>

#include <functional>
#include <map>
#include <string>
#include <vector>
#include <memory>

#include "system_modes/msg/mode_event.hpp"

using std::function;
using std::map;
using std::mutex;
using std::placeholders::_1;
using std::string;
using std::vector;

using lifecycle_msgs::msg::State;
using lifecycle_msgs::msg::Transition;
using lifecycle_msgs::msg::TransitionEvent;
using lifecycle_msgs::srv::GetState;

using system_modes::msg::ModeEvent;
using system_modes::srv::GetMode;

using namespace std::chrono_literals;
using shared_mutex = std::shared_timed_mutex;

namespace system_modes
{

ModeObserver::ModeObserver(std::weak_ptr<rclcpp::Node> handle)
: node_handle_(handle)
{
}

StateAndMode
ModeObserver::get(const std::string & part_name)
{
  std::shared_lock<shared_mutex> lock(this->mutex_);
  try {
    return cache_.at(part_name);
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';
    return StateAndMode();
  }
}

void
ModeObserver::observe(const std::string & part_name)
{
  std::unique_lock<shared_mutex> lock(mutex_);

  cache_[part_name] = StateAndMode();

  // Initial try to get current state via service request
  std::string topic = part_name + "/get_state";
  auto gsrequest = std::make_shared<GetState::Request>();
  auto stateclient = node_handle_.lock()->create_client<GetState>(topic);
  if (stateclient->wait_for_service(std::chrono::microseconds(500))) {
    auto state = stateclient->async_send_request(gsrequest);
    if (rclcpp::spin_until_future_complete(node_handle_.lock(), state) ==
      rclcpp::FutureReturnCode::SUCCESS)
    {
      auto result = state.get();
      cache_[part_name].state = result->current_state.id;
    }
  }

  // Initial try to get current mode via service request
  topic = part_name + "/get_mode";
  auto gmrequest = std::make_shared<GetMode::Request>();
  auto modeclient = node_handle_.lock()->create_client<GetMode>(topic);
  if (modeclient->wait_for_service(std::chrono::microseconds(500))) {
    auto mode = modeclient->async_send_request(gmrequest);
    if (rclcpp::spin_until_future_complete(node_handle_.lock(), mode) ==
      rclcpp::FutureReturnCode::SUCCESS)
    {
      auto result = mode.get();
      cache_[part_name].mode = result->current_mode;
    }
  }

  // Set up transition subscriber and mode event subscriber for continuous observation
  topic = part_name + "/transition_event";
  function<void(TransitionEvent::SharedPtr)> transition_cb =
    bind(&ModeObserver::transition_callback, this, _1, part_name);
  auto state_sub = node_handle_.lock()->create_subscription<TransitionEvent>(
    topic,
    rclcpp::SystemDefaultsQoS(),
    transition_cb);
  state_subs_.emplace(part_name, state_sub);
  topic = part_name + "/mode_event";
  function<void(ModeEvent::SharedPtr)> mode_cb =
    bind(&ModeObserver::mode_event_callback, this, _1, part_name);
  auto mode_sub = node_handle_.lock()->create_subscription<ModeEvent>(
    topic,
    rclcpp::SystemDefaultsQoS(),
    mode_cb);
  mode_subs_.emplace(part_name, mode_sub);
}

void
ModeObserver::stop_observing(const std::string & part_name)
{
  std::unique_lock<shared_mutex> lock(mutex_);
  state_subs_.erase(part_name);
  mode_subs_.erase(part_name);
  cache_.erase(part_name);
}

void
ModeObserver::transition_callback(
  const TransitionEvent::SharedPtr msg,
  const string & part_name)
{
  std::unique_lock<shared_mutex> lock(mutex_);
  cache_[part_name].state = msg->goal_state.id;
}

void
ModeObserver::mode_event_callback(
  const ModeEvent::SharedPtr msg,
  const string & part_name)
{
  std::unique_lock<shared_mutex> lock(mutex_);
  cache_[part_name].mode = msg->goal_mode.label;
}

}  // namespace system_modes
