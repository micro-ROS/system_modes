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

#include <rclcpp/parameter.hpp>
#include <rclcpp/parameter_map.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rcl_yaml_param_parser/parser.h>

#include <map>
#include <mutex>
#include <string>
#include <vector>
#include <memory>
#include <utility>
#include <shared_mutex>

using std::endl;
using std::pair;
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
: nodes_(), nodes_target_(),
  systems_(), systems_target_(),
  modes_(),
  nodes_mutex_(), systems_mutex_(), modes_mutex_(), parts_mutex_(),
  nodes_target_mutex_(), systems_target_mutex_()
{
  this->read_modes_from_model(model_path);
}

void
ModeInference::update_state(const string & node, unsigned int state)
{
  std::unique_lock<shared_mutex> nlock(this->nodes_mutex_);

  auto it = this->nodes_.find(node);
  if (it != this->nodes_.end()) {
    string mode;
    if (state == State::PRIMARY_STATE_ACTIVE) {
      mode = this->nodes_[node].second;
    }
    this->nodes_[node] = make_pair(state, mode);
  }
}

void
ModeInference::update_mode(const string & node, const string & mode)
{
  std::unique_lock<shared_mutex> nlock(this->nodes_mutex_);

  auto it = this->nodes_.find(node);
  if (it != this->nodes_.end()) {
    this->nodes_[node] = make_pair(this->nodes_[node].first, mode);
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
ModeInference::update_target(const string & part, std::pair<unsigned int, string> mode)
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

std::pair<unsigned int, string>
ModeInference::get_target(const string & part)
{
  std::shared_lock<shared_mutex> ntlock(this->nodes_target_mutex_);
  std::shared_lock<shared_mutex> stlock(this->systems_target_mutex_);

  auto it = this->nodes_target_.find(part);
  auto its = this->systems_target_.find(part);
  if (it != this->nodes_target_.end()) {
    // we know this node
    return this->nodes_target_[part];
  } else if (its != this->systems_target_.end()) {
    // we know the system, probably from a mode manager
    return this->systems_target_[part];
  }

  // might be a system without explicit mode manager, trying to infer the mode
  throw std::out_of_range("No information about target for system or node '" + part + "'.");
}

std::pair<unsigned int, string>
ModeInference::get(const string & part)
{
  std::shared_lock<shared_mutex> nlock(this->nodes_mutex_);

  auto it = this->nodes_.find(part);
  if (it == this->nodes_.end()) {
    throw std::out_of_range("Unknown system or node '" + part + "'.");
  }

  if (this->nodes_[part].first == 0 && this->nodes_[part].second.empty()) {
    throw std::runtime_error("No solid information about state and mode of '" + part + "'.");
  }

  return this->nodes_[part];
}

std::pair<unsigned int, string>
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

std::pair<unsigned int, string>
ModeInference::infer_system(const string & part)
{
  unsigned int state = 0;  // unknown
  string mode = "";

  std::shared_lock<shared_mutex> mlock(this->modes_mutex_);
  auto default_mode = this->modes_[part][DEFAULT_MODE];
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
      if (stateAndMode.first == State::TRANSITION_STATE_ERRORPROCESSING) {
        return make_pair(State::TRANSITION_STATE_ERRORPROCESSING, "");
      }

      // all parts of same state?
      if (state == 0) {
        state = stateAndMode.first;
      } else if (state != stateAndMode.first) {
        // not the same, we can't say anything
        throw std::runtime_error(
                "Inconsistent information about parts of the system '" + part +
                "', inference failed.");
      }
    }
    return make_pair(state, "");
  }

  /** MAIN INFERENCE **/
  // state/mode-based
  unsigned int targetState = stateAndModeTarget->second.first;
  string targetMode = stateAndModeTarget->second.second;
  if (targetState == State::PRIMARY_STATE_INACTIVE) {
    // target: inactive
    for (auto part : default_mode->get_parts()) {
      auto stateAndMode = this->get_or_infer(part);

      if (stateAndMode.first == State::TRANSITION_STATE_ERRORPROCESSING) {
        // Note: If current entity was in an active mode that allowed a part to
        // be in error-processing (by dont-care) and the current entity is requested
        // to switch to inactive, then the actual state of the current entity will
        // go to error-processing until the mentioned part recovers.
        return make_pair(State::TRANSITION_STATE_ERRORPROCESSING, "");
      }

      // If at least one part is not 'inactive', yet => we are still deactivating
      if (stateAndMode.first != State::PRIMARY_STATE_INACTIVE &&
        stateAndMode.first != State::PRIMARY_STATE_UNCONFIGURED)
      {
        return make_pair(State::TRANSITION_STATE_DEACTIVATING, "");
      }
    }
    return make_pair(State::PRIMARY_STATE_INACTIVE, "");
  } else if (targetState == State::PRIMARY_STATE_ACTIVE) {
    ModeConstPtr mode;
    if (this->modes_[part].find(targetMode) != this->modes_[part].end()) {
      auto mode = this->modes_[part][targetMode];

      // target: active
      auto inTargetMode = true;

      for (auto partpart : mode->get_parts()) {
        auto stateAndMode = this->get_or_infer(partpart);
        auto targetStateAndMode = mode->get_part_mode(partpart);

        // TODO(anordman): consider DONT-CARE
        if (stateAndMode.first == State::TRANSITION_STATE_ERRORPROCESSING) {
          return make_pair(State::TRANSITION_STATE_ERRORPROCESSING, "");
        }

        // TODO(anordman): overly complictated. intent: we are in our target
        // mode, if actual and target state are the same OR they can be
        // considered same, i.e. unconfigured and inactive
        if (
          (stateAndMode.first != targetStateAndMode.first &&
          !(targetStateAndMode.first == State::PRIMARY_STATE_INACTIVE &&
          stateAndMode.first == State::PRIMARY_STATE_UNCONFIGURED)) ||
          (stateAndMode.first == State::PRIMARY_STATE_ACTIVE &&
          stateAndMode.second.compare(targetStateAndMode.second) != 0))
        {
          // not in target state, or active but not in target mode
          inTargetMode = false;
          continue;
        }
      }
      if (inTargetMode) {
        // Target state and target mode reached, all good!
        return make_pair(State::PRIMARY_STATE_ACTIVE, targetMode);
      }
    }

    // Check all remaining modes. Are we in any mode at all?
    for (auto mode : this->modes_[part]) {
      bool foundMode = true;
      for (auto partpart : default_mode->get_parts()) {
        auto targetStateAndMode = mode.second->get_part_mode(partpart);
        auto stateAndMode = this->get_or_infer(partpart);

        // TODO(anordman): consider DONT-CARE
        if (stateAndMode.first == State::TRANSITION_STATE_ERRORPROCESSING) {
          return make_pair(State::TRANSITION_STATE_ERRORPROCESSING, "");
        }

        if (stateAndMode.first != targetStateAndMode.first ||
          (stateAndMode.first == State::PRIMARY_STATE_ACTIVE &&
          stateAndMode.second.compare(targetStateAndMode.second) != 0))
        {
          // not in target state, or active but not in target mode
          foundMode = false;
          continue;
        }
      }
      if (foundMode) {
        // We are in a non-target mode, this means we are still activating
        return make_pair(State::TRANSITION_STATE_ACTIVATING, mode.first);
      }
    }

    return make_pair(State::TRANSITION_STATE_ACTIVATING, "?");
  }

  throw std::runtime_error("Inference failed.");
}

std::pair<unsigned int, string>
ModeInference::infer_node(const string & part)
{
  std::shared_lock<shared_mutex> mlock(this->modes_mutex_);
  std::shared_lock<shared_mutex> prlock(this->param_mutex_);

  auto default_mode = this->modes_[part][DEFAULT_MODE];
  if (!default_mode) {
    throw std::out_of_range(
            "Can't infer for node '" + part +
            "', missing default mode.");
  }

  // Do we know the target mode?
  try {
    auto target = this->get_target(part);
    string targetMode = target.second;

    if (!targetMode.empty()) {
      bool inTargetMode = true;

      // we know the target mode, so check this one first
      if (this->modes_[part].find(targetMode) != this->modes_[part].end()) {
        auto mode = this->modes_[part][targetMode];

        for (auto param : mode->get_parameter_names()) {
          if (!matching_parameters(
              mode->get_parameter(param),
              parameters_[part][param]))
          {
            inTargetMode = false;
            continue;
          }
        }
        if (inTargetMode) {
          return make_pair(State::PRIMARY_STATE_ACTIVE, targetMode);
        }
      }
    }
  } catch (...) {
    // safe to ignore, go on
  }

  // no target mode, so next we check the default mode
  bool inDefaultMode = true;
  auto defaultMode = this->modes_[part][DEFAULT_MODE];
  for (auto param : defaultMode->get_parameter_names()) {
    if (!matching_parameters(
        defaultMode->get_parameter(param),
        parameters_[part][param]))
    {
      inDefaultMode = false;
      continue;
    }
  }
  if (inDefaultMode) {
    return make_pair(State::PRIMARY_STATE_ACTIVE, DEFAULT_MODE);
  }

  // no target mode, not default mode, so we try our luck, infering any mode from parameters
  for (auto mode : this->modes_[part]) {
    auto m = this->modes_[part][mode.first];
    bool foundMode = true;
    for (auto param : defaultMode->get_parameter_names()) {
      if (!matching_parameters(
          m->get_parameter(param),
          parameters_[part][param]))
      {
        foundMode = false;
        continue;
      }
    }
    if (foundMode) {
      // We are in a non-target mode, this means we are still activating
      return make_pair(State::PRIMARY_STATE_ACTIVE, mode.first);
    }
  }

  throw std::runtime_error("Inference failed for node '" + part + "'.");
}

std::pair<unsigned int, string>
ModeInference::get_or_infer(const string & part)
{
  pair<unsigned int, string> stateAndMode;
  try {
    stateAndMode = this->get(part);
    if (stateAndMode.first != 0 && !stateAndMode.second.empty()) {
      return stateAndMode;
    }
  } catch (...) {
    // not a node, so try inference
  }

  try {
    auto stateAndModeInfer = this->infer(part);
    if (stateAndMode.first == 0 && !stateAndModeInfer.first == 0) {
      stateAndMode.first = stateAndModeInfer.first;
    }
    if (stateAndMode.second.empty() && !stateAndModeInfer.second.empty()) {
      stateAndMode.second = stateAndModeInfer.second;
    }
  } catch (...) {
    // ignore, inference is optional
  }

  if (stateAndMode.first == 0 && stateAndMode.second.empty()) {
    throw std::runtime_error("Not able to infer anything for part " + part);
  }

  return stateAndMode;
}

ModeConstPtr
ModeInference::get_mode(const string & part, const string & mode)
{
  std::shared_lock<shared_mutex> mlock(this->modes_mutex_);

  auto it = this->modes_.find(part);
  if (it != this->modes_.end()) {
    // We know this part
    auto its = this->modes_[part].find(mode);
    if (its != this->modes_[part].end()) {
      return this->modes_[part][mode];
    }
    return nullptr;
  }
  return nullptr;
}

std::vector<std::string>
ModeInference::get_available_modes(const std::string & part)
{
  std::vector<std::string> modes;
  for (auto mode : this->modes_[part]) {
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

          mode->set_part_mode(part_name, make_pair(state_id_(state), smode));
        }
        this->modes_[part_name].emplace(mode->get_name(), mode);

      } else {
        if (param.value_to_string().compare("node") != 0) {
          this->systems_.emplace(part_name, make_pair(0, ""));
        } else {
          this->nodes_.emplace(part_name, make_pair(0, ""));
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
  std::size_t foundr = param.get_name().rfind(".");
  if (foundr != std::string::npos) {
    param_name = param_name.substr(foundr + 1);
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
ModeInference::matching_parameters(const Parameter & target, const Parameter & actual)
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

}  // namespace system_modes
