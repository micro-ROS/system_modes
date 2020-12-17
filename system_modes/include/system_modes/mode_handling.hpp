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

#include <stdint.h>
#include <shared_mutex>
#include <rclcpp/parameter.hpp>
#include <rclcpp/parameter_map.hpp>

#include <map>
#include <mutex>
#include <string>
#include <memory>
#include <vector>
#include <utility>

#include "system_modes/mode.hpp"
#include "system_modes/mode_impl.hpp"

namespace system_modes
{

struct ModeRule
{
  std::string name;

  std::string system;
  StateAndMode system_target;

  std::string part;
  StateAndMode part_actual;

  StateAndMode new_system_target;
};

using RulesMap = std::map<std::string, ModeRule>;

class ModeHandling
{
public:
  explicit ModeHandling(const std::string & model_path);
  // cppcheck-suppress unknownMacro
  RCLCPP_DISABLE_COPY(ModeHandling)

  virtual ~ModeHandling() = default;
  virtual const std::vector<ModeRule> get_rules_for(
    const std::string & system,
    const StateAndMode & target);

protected:
  mutable std::shared_timed_mutex rules_mutex_;

private:
  std::map<std::string, RulesMap> rules_;

  virtual void read_rules_from_model(const std::string & model_path);
  virtual void add_rule(
    const std::string & part,
    const std::string & rule_name,
    const rclcpp::Parameter & rule_param);
};

}  // namespace system_modes
