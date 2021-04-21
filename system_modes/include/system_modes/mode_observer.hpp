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
#pragma once

#include <rclcpp/rclcpp.hpp>

#include <lifecycle_msgs/msg/transition_event.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

#include "system_modes/mode.hpp"
#include "system_modes_msgs/msg/mode_event.hpp"
#include "system_modes_msgs/srv/get_mode.hpp"

namespace system_modes
{

using std::map;
using std::mutex;
using std::string;

using lifecycle_msgs::msg::TransitionEvent;
using lifecycle_msgs::srv::GetState;
using rclcpp::node_interfaces::NodeBaseInterface;
using system_modes_msgs::msg::ModeEvent;
using system_modes_msgs::srv::GetMode;

/**
 * Mode observer provides a local system modes cache.
 *
 * The mode observer serves as a local system modes cache, instantiated by a node. The mode
 * observer will initially try to gain current states/modes from all observes entities and will
 * then continuously monitor transitions to keep up to date.
 */
class ModeObserver
{
public:
  explicit ModeObserver(std::weak_ptr<rclcpp::Node>);
  virtual ~ModeObserver() = default;

  /**
   * Getter for observed state and mode.
   *
   * Returns cached observed state and mode for requested system entity (system or node).
   */
  virtual StateAndMode
  get(const string & part_name);

  /**
   * Add part to list of observed parts.
   */
  virtual void
  observe(const string & part_name);

  /**
   * Remove part from list of observed parts.
   */
  virtual void
  stop_observing(const string & part_name);

protected:
  virtual void
  transition_callback(
    const TransitionEvent::SharedPtr msg,
    const string & part_name);

  virtual void
  mode_event_callback(
    const ModeEvent::SharedPtr msg,
    const string & part_name);

private:
  std::weak_ptr<rclcpp::Node> node_handle_;
  map<string, StateAndMode> cache_;
  mutable std::shared_timed_mutex mutex_;

  map<string, std::shared_ptr<rclcpp::Subscription<TransitionEvent>>> state_subs_;
  map<string, std::shared_ptr<rclcpp::Subscription<ModeEvent>>> mode_subs_;
};

}  // namespace system_modes
