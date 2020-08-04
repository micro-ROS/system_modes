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
#include "system_modes/mode_impl.hpp"

#include <map>
#include <string>
#include <vector>
#include <utility>
#include <exception>

#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>

using std::map;
using std::pair;
using std::mutex;
using std::string;
using std::vector;
using std::to_string;
using std::lock_guard;
using std::out_of_range;
using rclcpp::Parameter;
using lifecycle_msgs::msg::State;
using lifecycle_msgs::msg::Transition;

namespace system_modes
{

ModeImpl::ModeImpl(const string & mode_name)
: name_(mode_name), param_(), part_modes_(), mutex_()
{
  if (mode_name.empty()) {
    throw std::invalid_argument("Mode name can't be empty.");
  }
}

string
ModeImpl::get_name() const
{
  return this->name_;
}

Parameter
ModeImpl::get_parameter(const string & param_name) const
{
  Parameter parameter;
  if (this->get_parameter(param_name, parameter)) {
    return parameter;
  } else {
    throw out_of_range("Parameter '" + param_name + "' not set");
  }
}

bool
ModeImpl::get_parameter(const string & param_name, Parameter & parameter) const
{
  lock_guard<mutex> lock(this->mutex_);

  if (this->param_.count(param_name)) {
    parameter = this->param_.at(param_name);
    return true;
  } else {
    return false;
  }
}

vector<string>
ModeImpl::get_parameter_names() const
{
  lock_guard<mutex> lock(mutex_);

  vector<string> results;
  for (auto p : this->param_) {
    results.push_back(p.first);
  }
  return results;
}

const vector<Parameter>
ModeImpl::get_parameters() const
{
  vector<Parameter> params;
  for (auto it = this->param_.begin(); it != this->param_.end(); ++it) {
    params.push_back(it->second);
  }
  return params;
}

void
ModeImpl::add_parameter(const Parameter & parameter)
{
  lock_guard<mutex> lock(this->mutex_);

  this->param_.emplace(parameter.get_name(), parameter);
}

void
ModeImpl::add_parameters(const vector<Parameter> & parameters)
{
  lock_guard<mutex> lock(this->mutex_);

  for (auto p : parameters) {
    this->param_[p.get_name()] = p;
  }
}

void
ModeImpl::set_parameter(const Parameter & parameter)
{
  string param_name = parameter.get_name();
  std::size_t foundr = parameter.get_name().rfind("ros__parameters");
  if (foundr != string::npos) {
    param_name = parameter.get_name().substr(foundr + strlen("ros__parameters") + 1);
  }

  if (this->param_.find(param_name) == this->param_.end()) {
    throw out_of_range(
            "Parameter '" + param_name + "' not available in mode '" +
            this->name_ + "', has to be present in default mode.");
  }

  this->param_[param_name] = parameter;
}

void
ModeImpl::set_parameters(const vector<Parameter> & parameters)
{
  for (auto p : parameters) {
    this->set_parameter(p);
  }
}

void
ModeImpl::add_part_mode(
  const string & part,
  const StateAndMode stateAndMode)
{
  this->part_modes_[part] = StateAndMode(stateAndMode.state, stateAndMode.mode);
}

void
ModeImpl::set_part_mode(
  const string & part,
  const StateAndMode stateAndMode)
{
  if (this->part_modes_.find(part) == this->part_modes_.end()) {
    throw out_of_range(
            "Part mode for part '" + part + "' not available in mode '" +
            this->name_ + "', has to be present in default mode.");
  }

  this->add_part_mode(part, stateAndMode);
}

const vector<string>
ModeImpl::get_parts() const
{
  vector<string> results;
  for (auto p : this->part_modes_) {
    results.push_back(p.first);
  }
  return results;
}

const StateAndMode
ModeImpl::get_part_mode(const string & part) const
{
  if (this->part_modes_.count(part)) {
    return this->part_modes_.at(part);
  } else {
    throw out_of_range(
            "Can't receive modes for part '" + part +
            "', part not specified.");
  }
}

// TODO(anordman): Can we get this from the rcl default state machine?
static const map<unsigned int, string> STATES_ = {
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

static const map<unsigned int, string> TRANSITIONS_ = {
  {Transition::TRANSITION_CREATE, "create"},
  {Transition::TRANSITION_CONFIGURE, "configure"},
  {Transition::TRANSITION_CLEANUP, "cleanup"},
  {Transition::TRANSITION_ACTIVATE, "activate"},
  {Transition::TRANSITION_DEACTIVATE, "deactivate"},
  {Transition::TRANSITION_INACTIVE_SHUTDOWN, "inactive_shutdown"},
  {Transition::TRANSITION_ACTIVE_SHUTDOWN, "active_shutdown"},
  {Transition::TRANSITION_DESTROY, "destroy"}
};

static const map<unsigned int, unsigned int> GOAL_STATES_ = {
  {Transition::TRANSITION_CREATE, State::PRIMARY_STATE_UNCONFIGURED},
  {Transition::TRANSITION_CONFIGURE, State::PRIMARY_STATE_INACTIVE},
  {Transition::TRANSITION_CLEANUP, State::PRIMARY_STATE_UNCONFIGURED},
  {Transition::TRANSITION_ACTIVATE, State::PRIMARY_STATE_ACTIVE},
  {Transition::TRANSITION_DEACTIVATE, State::PRIMARY_STATE_INACTIVE},
  {Transition::TRANSITION_INACTIVE_SHUTDOWN, State::PRIMARY_STATE_FINALIZED},
  {Transition::TRANSITION_ACTIVE_SHUTDOWN, State::PRIMARY_STATE_FINALIZED}
};

const string
state_label_(unsigned int state_id)
{
  try {
    return STATES_.at(state_id);
  } catch (...) {
    return "unknown";
  }
}

unsigned int
state_id_(const string & state_label)
{
  for (auto id : STATES_) {
    if (id.second.compare(state_label) == 0) {
      return id.first;
    }
  }
  return 0;
}

const string
transition_label_(unsigned int transition_id)
{
  try {
    return TRANSITIONS_.at(transition_id);
  } catch (...) {
    throw out_of_range(string("Unknown transition id ") + to_string(transition_id));
  }
}

unsigned int
transition_id_(const string & transition_label)
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
