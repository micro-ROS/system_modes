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

namespace system_modes
{

class ModeHandling
{
public:
  explicit ModeHandling(const std::string & model_path);
  RCLCPP_DISABLE_COPY(ModeHandling)

  virtual ~ModeHandling() = default;

protected:
  virtual void add_rule(const std::string & part, const rclcpp::Parameter & param);

  mutable std::shared_timed_mutex rules_mutex_;

private:
  std::map<std::string, ModeMap> rules_;
};

}  // namespace system_modes
