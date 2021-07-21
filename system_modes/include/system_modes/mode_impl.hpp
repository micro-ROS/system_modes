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

#include <rclcpp/node_interfaces/node_parameters.hpp>
#include <rclcpp/parameter_map.hpp>

#include <lifecycle_msgs/msg/state.hpp>

#include <map>
#include <mutex>
#include <vector>
#include <string>
#include <memory>
#include <utility>

#include "system_modes/visibility_control.hpp"

using lifecycle_msgs::msg::State;

namespace system_modes
{

static const char DEFAULT_MODE[] = "__DEFAULT__";


SYSTEM_MODES_PUBLIC
unsigned int
state_id_(const std::string & state_label);

SYSTEM_MODES_PUBLIC
const std::string
state_label_(unsigned int state_id);

SYSTEM_MODES_PUBLIC
unsigned int
transition_id_(const std::string & transition_label);

SYSTEM_MODES_PUBLIC
const std::string
transition_label_(unsigned int transition_id);

SYSTEM_MODES_PUBLIC
unsigned int
goal_state_(unsigned int transition_id);

struct StateAndMode
{
  unsigned int state;
  std::string mode;

  SYSTEM_MODES_PUBLIC
  explicit StateAndMode(unsigned int newstate = 0, const std::string & newmode = "")
  {
    state = newstate;
    mode = newmode;
  }

  SYSTEM_MODES_PUBLIC
  bool
  operator==(const StateAndMode & cmp) const
  {
    if (cmp.state != state) {
      return false;
    } else if (cmp.state != State::PRIMARY_STATE_ACTIVE) {
      return true;
    }

    return cmp.mode.compare(mode) == 0 ||                            // same mode
           (cmp.mode.compare(DEFAULT_MODE) == 0 && mode.empty()) ||  // we consider empty and
           (mode.compare(DEFAULT_MODE) == 0 && cmp.mode.empty());    // DEFAULT_MODE the same
  }

  SYSTEM_MODES_PUBLIC
  bool
  operator!=(const StateAndMode & cmp) const
  {
    return !(*this == cmp);
  }

  SYSTEM_MODES_PUBLIC
  void
  from_string(const std::string & sam)
  {
    auto dot = sam.find(".");
    if (dot != std::string::npos) {
      state = state_id_(sam.substr(0, dot));
      mode = sam.substr(dot + 1);
    } else {
      state = state_id_(sam);
      mode = "";
    }
  }

  SYSTEM_MODES_PUBLIC
  std::string
  as_string() const
  {
    if (state != State::PRIMARY_STATE_ACTIVE || mode.empty()) {
      return state_label_(state);
    }
    return state_label_(state) + "." + mode;
  }

  bool unknown()
  {
    return state == State::PRIMARY_STATE_UNKNOWN;
  }
};

class ModeImpl
{
public:
  SYSTEM_MODES_PUBLIC
  explicit ModeImpl(const std::string & mode_name);

  SYSTEM_MODES_PUBLIC
  virtual
  ~ModeImpl() = default;

  SYSTEM_MODES_PUBLIC
  ModeImpl(const ModeImpl & copy) = delete;

  SYSTEM_MODES_PUBLIC
  virtual std::string
  get_name() const;

  SYSTEM_MODES_PUBLIC
  virtual void
  add_parameter(const rclcpp::Parameter & parameter);

  SYSTEM_MODES_PUBLIC
  virtual void
  add_parameters(const std::vector<rclcpp::Parameter> & parameters);

  SYSTEM_MODES_PUBLIC
  virtual void
  add_part_mode(
    const std::string & part,
    const StateAndMode stateAndMode);

  SYSTEM_MODES_PUBLIC
  virtual void
  set_parameter(const rclcpp::Parameter & parameter);

  SYSTEM_MODES_PUBLIC
  virtual void
  set_parameters(const std::vector<rclcpp::Parameter> & parameters);

  SYSTEM_MODES_PUBLIC
  virtual void
  set_part_mode(
    const std::string & part,
    const StateAndMode stateAndMode);

  SYSTEM_MODES_PUBLIC
  virtual std::vector<std::string>
  get_parameter_names() const;

  SYSTEM_MODES_PUBLIC
  virtual rclcpp::Parameter
  get_parameter(const std::string & param_name) const;

  SYSTEM_MODES_PUBLIC
  virtual bool
  get_parameter(const std::string & param_name, rclcpp::Parameter & parameter) const;

  SYSTEM_MODES_PUBLIC
  virtual const std::vector<rclcpp::Parameter>
  get_parameters() const;

  SYSTEM_MODES_PUBLIC
  virtual const std::vector<std::string>
  get_parts() const;

  SYSTEM_MODES_PUBLIC
  virtual const StateAndMode
  get_part_mode(const std::string & part) const;

protected:
  std::string name_;
  std::map<std::string, rclcpp::Parameter> param_;
  std::map<std::string, StateAndMode> part_modes_;
  mutable std::mutex mutex_;
};

}  // namespace system_modes
