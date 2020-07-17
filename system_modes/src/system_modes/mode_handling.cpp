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
using std::make_pair;
using std::lock_guard;
using rclcpp::Parameter;
using rclcpp::ParameterMap;
using lifecycle_msgs::msg::State;

using shared_mutex = std::shared_timed_mutex;

namespace system_modes
{

ModeHandling::ModeHandling(const string & model_path)
{
  rcl_params_t * yaml_params = rcl_yaml_node_struct_init(rcl_get_default_allocator());
  if (!rcl_parse_yaml_file(model_path.c_str(), yaml_params)) {
    throw std::runtime_error("Failed to parse parameters " + model_path);
  }

  rclcpp::ParameterMap param_map = rclcpp::parameter_map_from(yaml_params);
  rcl_yaml_node_struct_fini(yaml_params);

  ParameterMap::iterator it;
  for (it = param_map.begin(); it != param_map.end(); it++) {
    string part_name(it->first.substr(1));

    auto itm = this->rules_.find(part_name);
    if (itm == this->rules_.end()) {
      this->rules_.emplace(part_name, ModeMap());
    }

    for (auto & param : it->second) {
      std::size_t foundm = param.get_name().find("rules.");
      if (foundm != string::npos) {
        std::size_t foundmr = param.get_name().find(".", 6);
        if (foundmr != string::npos) {
          this->add_rule(part_name, param);
        } else {
          continue;
        }
        } else {
          continue;
        }
    }
  }
}

void
ModeHandling::add_rule(const std::string & part, const Parameter & param)
{
  std::unique_lock<shared_mutex> mlock(this->rules_mutex_);
  throw std::runtime_error("ModeHandling::add_rule() not yet implemented");
}

}  // namespace system_modes
