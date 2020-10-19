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
#include "system_modes/mode_handling.hpp"

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
using std::lock_guard;
using rclcpp::Parameter;
using rclcpp::ParameterMap;
using rclcpp::ParameterType;
using lifecycle_msgs::msg::State;

using shared_mutex = std::shared_timed_mutex;

namespace system_modes
{

ModeHandling::ModeHandling(const string & model_path)
{
  this->read_rules_from_model(model_path);
}

void
ModeHandling::read_rules_from_model(const string & model_path)
{
  rcl_params_t * yaml_params = rcl_yaml_node_struct_init(rcl_get_default_allocator());
  if (!rcl_parse_yaml_file(model_path.c_str(), yaml_params)) {
    throw std::runtime_error("Failed to parse rules from " + model_path);
  }

  rclcpp::ParameterMap param_map = rclcpp::parameter_map_from(yaml_params);
  rcl_yaml_node_struct_fini(yaml_params);

  ParameterMap::iterator it;
  for (it = param_map.begin(); it != param_map.end(); it++) {
    string part_name(it->first.substr(1));
    for (auto & param : it->second) {
      string param_name(param.get_name());

      if (param_name.find("rules.") != string::npos) {
        this->add_rule(part_name, param_name.substr(6), param);
      }
    }
  }
}

void
ModeHandling::add_rule(
  const string & part,
  const string & rule_name,
  const rclcpp::Parameter & rule_param)
{
  std::unique_lock<shared_mutex> mlock(this->rules_mutex_);

  // Rule specification
  std::size_t split = rule_name.find(".");
  if (split == string::npos) {
    throw std::runtime_error("ModeHandling::add_rule() can't parse rule.");
  }
  string rule_spec = rule_name.substr(split + 1);
  string rule_name_ = rule_name.substr(0, split);

  if (rule_spec.compare("if_target") != 0 &&
    rule_spec.compare("if_part") != 0 &&
    rule_spec.compare("new_target") != 0)
  {
    throw std::runtime_error("ModeHandling::add_rule() can't parse rule spec.");
  }

  if (rule_param.get_type() != ParameterType::PARAMETER_STRING &&
    rule_param.get_type() != ParameterType::PARAMETER_STRING_ARRAY)
  {
    throw std::runtime_error("ModeHandling::add_rule() rule is neither string nor string array.");
  }

  // Insert rule if not existing already
  this->rules_.insert(std::make_pair(part, RulesMap()));
  auto it = this->rules_[part].insert(std::make_pair(rule_name_, ModeRule()));
  auto rule = it.first->second;

  rule.system = part;
  if (rule_spec.compare("if_target") == 0) {
    if (rule_param.get_type() != ParameterType::PARAMETER_STRING) {
      throw std::runtime_error("ModeHandling::parse_rule() if_target expects string.");
    }
    rule.name = rule_name_;
    rule.system_target.from_string(rule_param.as_string());
  } else if (rule_spec.compare("if_part") == 0) {
    if (rule_param.get_type() != ParameterType::PARAMETER_STRING_ARRAY) {
      throw std::runtime_error("ModeHandling::parse_rule() if_part expects string array.");
    }
    auto spec = rule_param.as_string_array();
    rule.name = rule_name_;
    rule.part = spec[0];
    rule.part_actual.from_string(spec[1]);
  } else if (rule_spec.compare("new_target") == 0) {
    if (rule_param.get_type() != ParameterType::PARAMETER_STRING) {
      throw std::runtime_error("ModeHandling::parse_rule() new_target expects string.");
    }
    rule.name = rule_name_;
    rule.new_system_target.from_string(rule_param.as_string());
  }
  this->rules_[part][rule_name_] = rule;
}

const std::vector<ModeRule>
ModeHandling::get_rules_for(const std::string & system, const StateAndMode & target)
{
  std::vector<ModeRule> rules;
  try {
    auto rulesmap = this->rules_[system];
    for (auto rule : rulesmap) {
      if (target == rule.second.system_target) {
        rules.push_back(rule.second);
      }
    }
  } catch (...) {
    // no problem, if we don't find any rule
  }

  return rules;
}

}  // namespace system_modes
