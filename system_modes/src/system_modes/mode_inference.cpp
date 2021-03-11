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
#include "system_modes/mode_inference.hpp"

#include <shared_mutex>

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/parameter_map.hpp>
#include <rcl_yaml_param_parser/parser.h>

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

using std::endl;
using std::mutex;
using std::string;
using std::make_pair;
using std::lock_guard;
using rclcpp::Parameter;
using rclcpp::ParameterMap;
using lifecycle_msgs::msg::State;

using shared_mutex = std::shared_timed_mutex;

namespace system_modes
{

ModeInference::ModeInference(const string & model_path)
: mode_handling_(new ModeHandling(model_path)),
  nodes_(), nodes_target_(),
  systems_(), systems_target_(),
  modes_(),
  nodes_mutex_(), systems_mutex_(), modes_mutex_(), parts_mutex_(),
  nodes_target_mutex_(), systems_target_mutex_()
{
  this->read_modes_from_model(model_path);
}

void
ModeInference::update(const string & node, const StateAndMode & sm)
{
  this->update_state(node, sm.state);
  if (sm.state == State::PRIMARY_STATE_ACTIVE) {
    this->update_mode(node, sm.mode);
  } else {
    this->update_mode(node, "");
  }
}

void
ModeInference::update_state(const string & node, unsigned int state)
{
  std::unique_lock<shared_mutex> nlock(this->nodes_mutex_);

  auto it = this->nodes_.find(node);
  if (it != this->nodes_.end()) {
    string mode;
    if (state == State::PRIMARY_STATE_ACTIVE) {
      mode = this->nodes_[node].mode;
    }
    this->nodes_[node] = StateAndMode(state, mode);
  } else {
    // TODO(anordman): Explicit message, when trying to update system
    throw std::out_of_range(
            "Can't update state of '" + node + "', unknown node.");
  }
}

void
ModeInference::update_mode(const string & node, const string & mode)
{
  std::unique_lock<shared_mutex> nlock(this->nodes_mutex_);

  auto it = this->nodes_.find(node);
  if (it != this->nodes_.end()) {
    this->nodes_[node] = StateAndMode(this->nodes_[node].state, mode);
  } else {
    // TODO(anordman): Explicit message, when trying to update system
    throw std::out_of_range(
            "Can't update mode of '" + node + "', unknown node.");
  }
}

void
ModeInference::update_param(const string & node, Parameter & param)
{
  std::unique_lock<shared_mutex> nlock(this->param_mutex_);

  // TODO(anordman): Think about, how to handle the leading slash - never? always?
  string nodename = node;
  const auto start = nodename.find_first_not_of("/");
  if (start != std::string::npos) {
    nodename = nodename.substr(start);
  }

  std::map<string, Parameter> newmap;
  this->parameters_.emplace(nodename, newmap);
  this->parameters_[nodename][param.get_name()] = param;
}

void
ModeInference::update_target(const string & part, StateAndMode mode)
{
  std::shared_lock<shared_mutex> nlock(this->nodes_mutex_);
  std::unique_lock<shared_mutex> ntlock(this->nodes_target_mutex_);
  auto it = this->nodes_.find(part);
  if (it != this->nodes_.end()) {
    this->nodes_target_[part] = mode;
  }

  std::shared_lock<shared_mutex> slock(this->systems_mutex_);
  std::unique_lock<shared_mutex> stlock(this->systems_target_mutex_);
  auto its = this->systems_.find(part);
  if (its != this->systems_.end()) {
    this->systems_target_[part] = mode;
  }
}

StateAndMode
ModeInference::get_target(const string & part) const
{
  std::shared_lock<shared_mutex> ntlock(this->nodes_target_mutex_);
  std::shared_lock<shared_mutex> stlock(this->systems_target_mutex_);

  auto it = this->nodes_target_.find(part);
  auto its = this->systems_target_.find(part);
  if (it != this->nodes_target_.end()) {
    // we know this node
    return this->nodes_target_.at(part);
  } else if (its != this->systems_target_.end()) {
    // we know the system, probably from a mode manager
    return this->systems_target_.at(part);
  }

  // might be a system without explicit mode manager, trying to infer the mode
  throw std::out_of_range("No information about target for system or node '" + part + "'.");
}

StateAndMode
ModeInference::get(const string & part) const
{
  std::shared_lock<shared_mutex> nlock(this->nodes_mutex_);

  auto it = this->nodes_.find(part);
  if (it == this->nodes_.end()) {
    throw std::out_of_range("Unknown system or node '" + part + "'.");
  }

  if (this->nodes_.at(part).state == 0 && this->nodes_.at(part).mode.empty()) {
    throw std::runtime_error("No solid information about state and mode of '" + part + "'.");
  }

  auto sam = this->nodes_.at(part);
  if (sam.state != State::PRIMARY_STATE_ACTIVE &&
    sam.state != State::TRANSITION_STATE_ACTIVATING)
  {
    sam.mode = "";
  }

  return sam;
}

StateAndMode
ModeInference::infer(const string & part)
{
  std::shared_lock<shared_mutex> slock(this->systems_mutex_);
  std::shared_lock<shared_mutex> nlock(this->nodes_mutex_);
  string mode;

  auto sit = this->systems_.find(part);
  if (sit != this->systems_.end()) {
    // Can only infer for known systems
    return this->infer_system(part);
  }

  auto nit = this->nodes_.find(part);
  if (nit != this->nodes_.end()) {
    // Can only infer for known systems
    return this->infer_node(part);
  } else {
    throw std::out_of_range("Inference failed for unknown system or node: " + part);
  }
}

StateAndMode
ModeInference::infer_system(const string & part)
{
  unsigned int state = 0;  // unknown
  string mode = "";

  std::shared_lock<shared_mutex> mlock(this->modes_mutex_);
  auto default_mode = this->modes_.at(part).at(DEFAULT_MODE);
  if (!default_mode) {
    throw std::out_of_range(
            "Can't infer for system '" + part +
            "', missing default mode.");
  }

  // If we don't know the target state/mode, we can hardly infer anything
  std::shared_lock<shared_mutex> stlock(this->systems_target_mutex_);
  auto stateAndModeTarget = this->systems_target_.find(part);
  if (stateAndModeTarget == this->systems_target_.end() || stateAndModeTarget->first.empty()) {
    for (auto itspart : default_mode->get_parts()) {
      auto stateAndMode = this->get_or_infer(itspart);

      // error-processing?
      if (stateAndMode.state == State::TRANSITION_STATE_ERRORPROCESSING) {
        this->systems_[part] = StateAndMode(State::TRANSITION_STATE_ERRORPROCESSING, "");
        return StateAndMode(State::TRANSITION_STATE_ERRORPROCESSING, "");
      }

      // all parts of same state?
      if (state == 0) {
        state = stateAndMode.state;
      } else if (state != stateAndMode.state) {
        // not the same, we can't say anything
        throw std::runtime_error(
                "Inconsistent information about parts of the system '" + part +
                "', inference failed.");
      }
    }
    return StateAndMode(state, "");
  }

  /** MAIN INFERENCE **/
  // state/mode-based
  unsigned int targetState = stateAndModeTarget->second.state;
  string targetMode = stateAndModeTarget->second.mode;
  if (targetState == State::PRIMARY_STATE_INACTIVE) {
    // target: inactive
    for (auto part : default_mode->get_parts()) {
      auto stateAndMode = this->get_or_infer(part);

      if (stateAndMode.state == State::TRANSITION_STATE_ERRORPROCESSING) {
        // Note: If current entity was in an active mode that allowed a part to
        // be in error-processing (by dont-care) and the current entity is requested
        // to switch to inactive, then the actual state of the current entity will
        // go to error-processing until the mentioned part recovers.
        this->systems_[part] = StateAndMode(State::TRANSITION_STATE_ERRORPROCESSING, "");
        return StateAndMode(State::TRANSITION_STATE_ERRORPROCESSING, "");
      }

      // If at least one part is not 'inactive', yet => we are still deactivating
      if (stateAndMode.state != State::PRIMARY_STATE_INACTIVE &&
        stateAndMode.state != State::PRIMARY_STATE_UNCONFIGURED)
      {
        this->systems_[part] = StateAndMode(State::TRANSITION_STATE_DEACTIVATING, "");
        return StateAndMode(State::TRANSITION_STATE_DEACTIVATING, "");
      }
    }
    return StateAndMode(State::PRIMARY_STATE_INACTIVE, "");
  } else if (targetState == State::PRIMARY_STATE_ACTIVE) {
    ModeConstPtr mode;
    if (this->modes_.at(part).find(targetMode) != this->modes_.at(part).end()) {
      auto mode = this->modes_.at(part).at(targetMode);

      // target: active
      auto inTargetMode = true;

      for (auto partpart : mode->get_parts()) {
        auto stateAndMode = this->get_or_infer(partpart);
        auto targetStateAndMode = mode->get_part_mode(partpart);

        // TODO(anordman): consider DONT-CARE
        if (stateAndMode.state == State::TRANSITION_STATE_ERRORPROCESSING) {
          this->systems_[part] = StateAndMode(State::TRANSITION_STATE_ERRORPROCESSING, "");
          return StateAndMode(State::TRANSITION_STATE_ERRORPROCESSING, "");
        }

        // TODO(anordman): overly complictated. intent: we are in our target
        // mode, if actual and target state are the same OR they can be
        // considered same, i.e. unconfigured and inactive
        if (
          (stateAndMode.state != targetStateAndMode.state &&
          !(targetStateAndMode.state == State::PRIMARY_STATE_INACTIVE &&
          stateAndMode.state == State::PRIMARY_STATE_UNCONFIGURED)) ||
          (stateAndMode.state == State::PRIMARY_STATE_ACTIVE &&
          stateAndMode.mode.compare(targetStateAndMode.mode) != 0))
        {
          // not in target state, or active but not in target mode
          inTargetMode = false;
          continue;
        }
      }
      if (inTargetMode) {
        // Target state and target mode reached, all good!
        this->systems_[part] = StateAndMode(State::PRIMARY_STATE_ACTIVE, targetMode);
        return StateAndMode(State::PRIMARY_STATE_ACTIVE, targetMode);
      }
    }

    // Check all remaining modes. Are we in any mode at all?
    for (auto mode : this->modes_.at(part)) {
      bool foundMode = true;
      for (auto partpart : default_mode->get_parts()) {
        auto targetStateAndMode = mode.second->get_part_mode(partpart);
        auto stateAndMode = this->get_or_infer(partpart);

        // TODO(anordman): consider DONT-CARE
        if (stateAndMode.state == State::TRANSITION_STATE_ERRORPROCESSING) {
          this->systems_[part] = StateAndMode(State::TRANSITION_STATE_ERRORPROCESSING, "");
          return StateAndMode(State::TRANSITION_STATE_ERRORPROCESSING, "");
        }

        if (stateAndMode.state != targetStateAndMode.state ||
          (stateAndMode.state == State::PRIMARY_STATE_ACTIVE &&
          stateAndMode.mode.compare(targetStateAndMode.mode) != 0))
        {
          // not in target state, or active but not in target mode
          foundMode = false;
          continue;
        }
      }
      if (foundMode) {
        // We are in a non-target mode, this means we are still activating
        this->systems_[part] = StateAndMode(State::TRANSITION_STATE_ACTIVATING, mode.first);
        return StateAndMode(State::TRANSITION_STATE_ACTIVATING, mode.first);
      }
    }

    this->systems_[part] = StateAndMode(State::TRANSITION_STATE_ACTIVATING, "");
    return StateAndMode(State::TRANSITION_STATE_ACTIVATING, "");
  }

  throw std::runtime_error("Inference failed.");
}

StateAndMode
ModeInference::infer_node(const string & part)
{
  std::shared_lock<shared_mutex> mlock(this->modes_mutex_);
  std::shared_lock<shared_mutex> prlock(this->param_mutex_);

  auto default_mode = this->modes_.at(part).at(DEFAULT_MODE);
  if (!default_mode) {
    throw std::out_of_range(
            "Can't infer for node '" + part +
            "', missing default mode.");
  }

  // Do we know the target mode?
  try {
    auto target = this->get_target(part);
    auto targetState = target.state;
    auto targetMode = target.mode;

    if (targetState != State::PRIMARY_STATE_ACTIVE || !targetMode.empty()) {
      bool inTargetMode = true;

      // we know the target mode, so check this one first
      if (this->modes_.at(part).find(targetMode) != this->modes_.at(part).end()) {
        auto mode = this->modes_.at(part).at(targetMode);

        for (auto param : mode->get_parameter_names()) {
          if (!matching_parameters(
              mode->get_parameter(param),
              parameters_.at(part).at(param)))
          {
            inTargetMode = false;
            continue;
          }
        }
        if (inTargetMode) {
          return StateAndMode(State::PRIMARY_STATE_ACTIVE, targetMode);
        }
      }
    }
  } catch (...) {
    // safe to ignore, go on
  }

  // no target mode, so next we check the default mode
  bool inDefaultMode = true;
  auto defaultMode = this->modes_.at(part).at(DEFAULT_MODE);
  for (auto param : defaultMode->get_parameter_names()) {
    if (!matching_parameters(
        defaultMode->get_parameter(param),
        parameters_.at(part).at(param)))
    {
      inDefaultMode = false;
      continue;
    }
  }
  if (inDefaultMode) {
    return StateAndMode(State::PRIMARY_STATE_ACTIVE, DEFAULT_MODE);
  }

  // no target mode, not default mode, so we try our luck, infering any mode from parameters
  for (auto mode : this->modes_.at(part)) {
    auto m = this->modes_.at(part).at(mode.first);
    bool foundMode = true;
    for (auto param : defaultMode->get_parameter_names()) {
      if (!matching_parameters(
          m->get_parameter(param),
          parameters_.at(part).at(param)))
      {
        foundMode = false;
        continue;
      }
    }
    if (foundMode) {
      // We are in a non-target mode, this means we are still activating
      return StateAndMode(State::PRIMARY_STATE_ACTIVE, mode.first);
    }
  }

  throw std::runtime_error("Inference failed for node '" + part + "'.");
}

StateAndMode
ModeInference::get_or_infer(const string & part)
{
  StateAndMode stateAndMode;
  try {
    stateAndMode = this->get(part);
    if (stateAndMode.state != 0 && !stateAndMode.mode.empty()) {
      return stateAndMode;
    }
  } catch (...) {
    // so try inference
  }

  try {
    auto stateAndModeInfer = this->infer(part);
    if (stateAndMode.state == 0 && !stateAndModeInfer.state == 0) {
      stateAndMode.state = stateAndModeInfer.state;
      stateAndMode.mode = stateAndModeInfer.mode;
    }
    if (stateAndMode.mode.empty() && !stateAndModeInfer.mode.empty()) {
      stateAndMode.mode = stateAndModeInfer.mode;
    }
  } catch (...) {
    // ignore, inference is optional
  }

  if (stateAndMode.state == 0 && stateAndMode.mode.empty()) {
    throw std::runtime_error("Not able to infer anything for part " + part);
  }
  if (stateAndMode.state != State::PRIMARY_STATE_ACTIVE) {
    stateAndMode.mode = "";
  }

  return stateAndMode;
}

ModeConstPtr
ModeInference::get_mode(const string & part, const string & mode) const
{
  std::shared_lock<shared_mutex> mlock(this->modes_mutex_);

  auto it = this->modes_.find(part);
  if (it != this->modes_.end()) {
    // We know this part
    auto its = this->modes_.at(part).find(mode);
    if (its != this->modes_.at(part).end()) {
      return this->modes_.at(part).at(mode);
    }
    return nullptr;
  }
  return nullptr;
}

std::vector<std::string>
ModeInference::get_available_modes(const std::string & part) const
{
  std::vector<std::string> modes;
  for (auto mode : this->modes_.at(part)) {
    modes.push_back(mode.first);
  }
  return modes;
}

void
ModeInference::read_modes_from_model(const string & model_path)
{
  std::unique_lock<shared_mutex> nlock(this->nodes_mutex_);
  std::unique_lock<shared_mutex> slock(this->systems_mutex_);
  std::unique_lock<shared_mutex> mlock(this->modes_mutex_);
  std::unique_lock<shared_mutex> plock(this->parts_mutex_);

  rcl_params_t * yaml_params = rcl_yaml_node_struct_init(rcl_get_default_allocator());
  if (!rcl_parse_yaml_file(model_path.c_str(), yaml_params)) {
    throw std::runtime_error("Failed to parse parameters " + model_path);
  }

  rclcpp::ParameterMap param_map = rclcpp::parameter_map_from(yaml_params);
  rcl_yaml_node_struct_fini(yaml_params);

  ParameterMap::iterator it;
  for (it = param_map.begin(); it != param_map.end(); it++) {
    string part_name(it->first.substr(1));

    auto itm = this->modes_.find(part_name);
    if (itm == this->modes_.end()) {
      this->modes_.emplace(part_name, ModeMap());
    }

    ModeBasePtr mode;
    DefaultModePtr default_mode;
    for (auto & param : it->second) {
      if (param.get_name().compare("type") != 0) {
        string mode_name;

        // Parse mode definitions
        std::size_t foundm = param.get_name().find("modes.");
        if (foundm != string::npos) {
          std::size_t foundmr = param.get_name().find(".", 6);
          if (foundmr != string::npos) {
            mode_name = param.get_name().substr(foundm + 6, foundmr - foundm - 6);
          } else {
            continue;
          }
        } else {
          continue;
        }

        // valid parameter, add to mode map
        if (!mode || mode->get_name().compare(mode_name) != 0) {
          if (mode_name.compare(DEFAULT_MODE) != 0) {
            if (!default_mode) {
              throw std::runtime_error(
                      "Could not find default mode for mode '" +
                      mode_name + "'. Make sure, default mode is defined first.");
            }
            mode = std::make_shared<Mode>(mode_name, default_mode);
          } else {
            default_mode = std::make_shared<DefaultMode>();
            mode = default_mode;
          }
        }
        std::size_t found = param.get_name().find("ros__parameters");
        if (found != string::npos) {
          this->add_param_to_mode(mode, param);
        } else {
          // part mode
          string part_name = param.get_name();
          std::size_t foundr = param.get_name().rfind(".");
          if (foundr != string::npos) {
            part_name = param.get_name().substr(foundr + 1);
          }

          string state(param.value_to_string());
          string smode;
          foundr = param.value_to_string().rfind(".");
          if (foundr != string::npos) {
            state = param.value_to_string().substr(0, foundr);
            smode = param.value_to_string().substr(foundr + 1);
          }

          mode->set_part_mode(part_name, StateAndMode(state_id_(state), smode));
        }
        this->modes_[part_name].emplace(mode->get_name(), mode);

      } else {
        if (param.value_to_string().compare("node") != 0) {
          this->systems_.emplace(
            part_name,
            StateAndMode(State::PRIMARY_STATE_UNKNOWN, ""));
        } else {
          this->nodes_.emplace(
            part_name,
            StateAndMode(State::PRIMARY_STATE_UNKNOWN, ""));
        }
      }
    }
  }
}

void
ModeInference::add_param_to_mode(ModeBasePtr mode, const Parameter & param)
{
  // TODO(anordman): All of this has to be easier
  std::string param_name = param.get_name();
  std::size_t foundr = param.get_name().rfind("ros__parameters");
  if (foundr != std::string::npos) {
    param_name = param_name.substr(foundr + strlen("ros__parameters") + 1);
  }
  auto paramMsg = param.to_parameter_msg();
  paramMsg.name = param_name;
  auto cleanParameter = Parameter::from_parameter_msg(paramMsg);

  mode->set_parameter(cleanParameter);
}

const std::vector<string>
ModeInference::get_all_parts() const
{
  std::vector<string> result;
  for (auto system : this->get_systems()) {
    result.push_back(system);
  }
  for (auto node : this->get_nodes()) {
    result.push_back(node);
  }
  return result;
}

const std::vector<string>
ModeInference::get_nodes() const
{
  std::vector<string> result;
  for (auto node : this->nodes_) {
    result.push_back(node.first);
  }
  return result;
}

const std::vector<string>
ModeInference::get_systems() const
{
  std::vector<string> result;
  for (auto node : this->systems_) {
    result.push_back(node.first);
  }
  return result;
}

const std::vector<string>
ModeInference::get_all_parts_of(const string & system) const
{
  if (system.empty()) {
    return std::vector<string>();
  }
  return this->modes_.at(system).at(DEFAULT_MODE)->get_parts();
}

bool
ModeInference::matching_parameters(const Parameter & target, const Parameter & actual) const
{
  // TODO(anordman): consider DONTCARE and value ranges

  // This should not happen
  if (target.get_type() != actual.get_type()) {
    return false;
  }

  // string
  if (target.get_type() == rcl_interfaces::msg::ParameterType::PARAMETER_STRING &&
    target.as_string().compare(actual.as_string()) == 0)
  {
    return true;
  }
  // int
  if (target.get_type() == rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER &&
    target.as_int() == actual.as_int())
  {
    return true;
  }
  // bool
  if (target.get_type() == rcl_interfaces::msg::ParameterType::PARAMETER_BOOL &&
    target.as_bool() == actual.as_bool())
  {
    return true;
  }
  // double
  if (target.get_type() == rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE &&
    target.as_double() == actual.as_double())
  {
    return true;
  }
  // TODO(anordman): More types

  return false;
}

Deviation
ModeInference::infer_transitions()
{
  Deviation transitions;

  {
    std::unique_lock<shared_mutex> nlock(this->nodes_mutex_);
    std::unique_lock<shared_mutex> nclock(this->nodes_cache_mutex_);
    StatesMap::iterator it;
    for (it = nodes_.begin(); it != nodes_.end(); it++) {
      if (nodes_cache_.count(it->first) < 1) {
        nodes_cache_[it->first] = nodes_.at(it->first);
      }
      try {
        auto sm_current = infer_node(it->first);
        if (sm_current.state == State::PRIMARY_STATE_ACTIVE &&
          sm_current.mode.compare(nodes_cache_.at(it->first).mode) != 0)
        {
          // Detected a mode transition
          transitions[it->first] = make_pair(nodes_cache_.at(it->first), sm_current);

          // Cache newly inferred state and mode for next inference of transitions
          nodes_cache_[it->first] = sm_current;
        }
      } catch (...) {
        // inference may not work due to too little information
        continue;
      }
    }
  }

  {
    std::unique_lock<shared_mutex> slock(this->systems_mutex_);
    std::unique_lock<shared_mutex> sclock(this->systems_cache_mutex_);
    StatesMap::iterator it;
    for (it = systems_.begin(); it != systems_.end(); it++) {
      if (systems_cache_.count(it->first) < 1) {
        systems_cache_[it->first] = systems_.at(it->first);
      }
      try {
        auto sm_current = infer_system(it->first);
        if (sm_current != systems_cache_[it->first]) {
          // Detected a transition
          transitions[it->first] = make_pair(systems_cache_.at(it->first), sm_current);

          // Cache newly inferred state and mode for next inference of transitions
          systems_cache_[it->first] = sm_current;
        }
      } catch (...) {
        // inference may not work due to too little information
        continue;
      }
    }
  }

  return transitions;
}

Deviation
ModeInference::get_deviation()
{
  Deviation deviation;

  for (auto const & part : get_all_parts()) {
    try {
      auto actual = get_or_infer(part);
      auto target = get_target(part);
      if (actual != target) {
        deviation[part] = std::make_pair(target, actual);
      }
    } catch (...) {
      // We can't get deviations, if we can't infer the system state
    }
  }

  return deviation;
}

}  // namespace system_modes
