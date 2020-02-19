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
#include "system_modes/mode.hpp"

#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>

#include <map>
#include <string>
#include <vector>
#include <utility>

using std::string;
using std::vector;
using std::to_string;
using std::out_of_range;
using std::runtime_error;
using rclcpp::Parameter;
using lifecycle_msgs::msg::State;
using lifecycle_msgs::msg::Transition;

namespace system_modes
{

ModeBase::ModeBase(const string & mode_name)
: mode_impl_(mode_name)
{
}

string
ModeBase::get_name() const
{
  return this->mode_impl_.get_name();
}

Parameter
ModeBase::get_parameter(const string & param_name) const
{
  return this->mode_impl_.get_parameter(param_name);
}

vector<string>
ModeBase::get_parameter_names() const
{
  return this->mode_impl_.get_parameter_names();
}

const vector<Parameter>
ModeBase::get_parameters() const
{
  return this->mode_impl_.get_parameters();
}

const vector<string>
ModeBase::get_parts() const
{
  return this->mode_impl_.get_parts();
}

const StateAndMode
ModeBase::get_part_mode(const string & part) const
{
  return this->mode_impl_.get_part_mode(part);
}

string
ModeBase::print() const
{
  std::ostringstream os(std::ostringstream::out);
  os.precision(2);
  os << this->get_name() << "<";

  bool first = true;
  for (auto p : this->get_parameter_names()) {
    if (!first) {
      os << ", ";
    } else {
      first = false;
    }
    os << p << ":" << this->get_parameter(p).value_to_string();
  }
  first = true;
  for (auto p : this->get_parts()) {
    if (!first) {
      os << ", ";
    } else {
      first = false;
    }
    auto stateAndMode = this->get_part_mode(p);
    os << p << ":" << stateAndMode.state << stateAndMode.mode;
  }

  os << ">";
  return os.str();
}

DefaultMode::DefaultMode()
: ModeBase(DEFAULT_MODE)
{
}

void
DefaultMode::set_parameter(const Parameter & parameter)
{
  this->mode_impl_.add_parameter(parameter);
}

void
DefaultMode::set_parameters(const vector<Parameter> & parameters)
{
  this->mode_impl_.add_parameters(parameters);
}

void
DefaultMode::set_part_mode(
  const std::string & part,
  const StateAndMode stateAndMode)
{
  if (stateAndMode.mode.empty()) {
    this->mode_impl_.add_part_mode(part, StateAndMode(stateAndMode.state, DEFAULT_MODE));
  } else {
    this->mode_impl_.add_part_mode(part, stateAndMode);
  }
}

Mode::Mode(const string & mode_name, const DefaultModePtr default_mode)
: ModeBase(mode_name)
{
  if (!default_mode) {
    throw std::runtime_error("Default mode must not be empty.");
  }

  auto default_params = default_mode->get_parameter_names();
  for (auto p : default_params) {
    this->mode_impl_.add_parameter(default_mode->get_parameter(p));
  }

  auto default_parts = default_mode->get_parts();
  for (auto p : default_parts) {
    this->mode_impl_.add_part_mode(p, default_mode->get_part_mode(p));
  }
}

void
Mode::set_parameter(const Parameter & parameter)
{
  this->mode_impl_.set_parameter(parameter);
}

void
Mode::set_parameters(const vector<Parameter> & parameters)
{
  this->mode_impl_.set_parameters(parameters);
}

void
Mode::set_part_mode(
  const std::string & part,
  const StateAndMode stateAndMode)
{
  if (stateAndMode.mode.empty()) {
    this->mode_impl_.add_part_mode(part, StateAndMode(stateAndMode.state, DEFAULT_MODE));
  } else {
    this->mode_impl_.add_part_mode(part, stateAndMode);
  }
}

// TODO(anordman): Can we get this from the rcl default state machine?
static const std::map<unsigned int, string> STATES_ = {
  {State::PRIMARY_STATE_UNKNOWN, "unknown"},
  {State::PRIMARY_STATE_UNCONFIGURED, "unconfigured"},
  {State::PRIMARY_STATE_INACTIVE, "inactive"},
  {State::PRIMARY_STATE_ACTIVE, "active"},
  {State::PRIMARY_STATE_FINALIZED, "finalized"},
  {State::TRANSITION_STATE_CONFIGURING, "configuring"},
  {State::TRANSITION_STATE_CLEANINGUP, "cleaningup"},
  {State::TRANSITION_STATE_SHUTTINGDOWN, "shuttingdown"},
  {State::TRANSITION_STATE_ACTIVATING, "activating"},
  {State::TRANSITION_STATE_DEACTIVATING, "deactivating"},
  {State::TRANSITION_STATE_ERRORPROCESSING, "errorprocessing"}
};

static const std::map<unsigned int, string> TRANSITIONS_ = {
  {Transition::TRANSITION_CREATE, "create"},
  {Transition::TRANSITION_CONFIGURE, "configure"},
  {Transition::TRANSITION_CLEANUP, "cleanup"},
  {Transition::TRANSITION_ACTIVATE, "activate"},
  {Transition::TRANSITION_DEACTIVATE, "deactivate"},
  {Transition::TRANSITION_INACTIVE_SHUTDOWN, "inactive_shutdown"},
  {Transition::TRANSITION_ACTIVE_SHUTDOWN, "active_shutdown"},
  {Transition::TRANSITION_DESTROY, "destroy"}
};

static const std::map<unsigned int, unsigned int> GOAL_STATES_ = {
  {Transition::TRANSITION_CREATE, State::PRIMARY_STATE_UNCONFIGURED},
  {Transition::TRANSITION_CONFIGURE, State::PRIMARY_STATE_INACTIVE},
  {Transition::TRANSITION_CLEANUP, State::PRIMARY_STATE_UNCONFIGURED},
  {Transition::TRANSITION_ACTIVATE, State::PRIMARY_STATE_ACTIVE},
  {Transition::TRANSITION_DEACTIVATE, State::PRIMARY_STATE_INACTIVE},
  {Transition::TRANSITION_INACTIVE_SHUTDOWN, State::PRIMARY_STATE_FINALIZED},
  {Transition::TRANSITION_ACTIVE_SHUTDOWN, State::PRIMARY_STATE_FINALIZED}
};

const std::string
state_label_(unsigned int state_id)
{
  try {
    return STATES_.at(state_id);
  } catch (...) {
    return "unknown";
  }
}

unsigned int
state_id_(const std::string & state_label)
{
  for (auto id : STATES_) {
    if (id.second.compare(state_label) == 0) {
      return id.first;
    }
  }
  return 0;
}

const std::string
transition_label_(unsigned int transition_id)
{
  try {
    return TRANSITIONS_.at(transition_id);
  } catch (...) {
    throw out_of_range(string("Unknown transition id ") + to_string(transition_id));
  }
}

unsigned int
transition_id_(const std::string & transition_label)
{
  for (auto id : TRANSITIONS_) {
    if (id.second.compare(transition_label) == 0) {
      return id.first;
    }
  }
  throw out_of_range("Unknown transition " + transition_label);
}

unsigned int
goal_state_(unsigned int transition_id)
{
  try {
    return GOAL_STATES_.at(transition_id);
  } catch (...) {
    throw out_of_range(string("Unknown transition id ") + to_string(transition_id));
  }
}

}  // namespace system_modes
