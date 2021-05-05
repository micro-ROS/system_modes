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
#include "system_modes/mode_handling.hpp"
#include "system_modes/visibility_control.hpp"

namespace system_modes
{

typedef std::map<std::string, StateAndMode> StatesMap;
typedef std::map<std::string, std::map<std::string, rclcpp::Parameter>> ParametersMap;
typedef std::map<std::string, std::pair<StateAndMode, StateAndMode>> Deviation;

class ModeInference
{
public:
  SYSTEM_MODES_PUBLIC
  explicit
  ModeInference(const std::string & model_path);
  // cppcheck-suppress unknownMacro
  RCLCPP_DISABLE_COPY(ModeInference)

  SYSTEM_MODES_PUBLIC
  virtual const std::vector<std::string>
  get_all_parts() const;
  
  SYSTEM_MODES_PUBLIC
  virtual const std::vector<std::string>
  get_nodes() const;
  
  SYSTEM_MODES_PUBLIC
  virtual const std::vector<std::string>
  get_systems() const;
  
  SYSTEM_MODES_PUBLIC
  virtual const std::vector<std::string>
  get_all_parts_of(
    const std::string & system) const;

  SYSTEM_MODES_PUBLIC
  virtual void
  update(const std::string &, const StateAndMode &);
  
  SYSTEM_MODES_PUBLIC
  virtual void
  update_state(const std::string &, unsigned int);
  
  SYSTEM_MODES_PUBLIC
  virtual void
  update_mode(const std::string &, const std::string &);
  
  SYSTEM_MODES_PUBLIC
  virtual void
  update_param(const std::string &, rclcpp::Parameter &);
  
  SYSTEM_MODES_PUBLIC
  virtual void
  update_target(const std::string &, StateAndMode);

  SYSTEM_MODES_PUBLIC
  virtual StateAndMode
  get(const std::string & part) const;
  
  SYSTEM_MODES_PUBLIC
  virtual StateAndMode
  get_or_infer(const std::string & part);

  SYSTEM_MODES_PUBLIC
  virtual StateAndMode
  infer(const std::string & part);
  
  SYSTEM_MODES_PUBLIC
  virtual StateAndMode
  infer_node(const std::string & part);
  
  SYSTEM_MODES_PUBLIC
  virtual StateAndMode
  infer_system(const std::string & part);

  /**
   * Infers latest transitions of systems
   *
   * Returns map of last inferred transitions of systems into new states or
   * new modes. State transitions of nodes don't have to be inferred, as
   * nodes publish their state transitions. For nodes, we only need to infer
   * mode transitions.
   */
  SYSTEM_MODES_PUBLIC
  virtual Deviation
  infer_transitions();
  
  SYSTEM_MODES_PUBLIC
  virtual Deviation
  get_deviation();

  SYSTEM_MODES_PUBLIC
  virtual StateAndMode
  get_target(const std::string & part) const;
  
  SYSTEM_MODES_PUBLIC
  virtual ModeConstPtr
  get_mode(const std::string & part, const std::string & mode) const;
  
  SYSTEM_MODES_PUBLIC
  virtual std::vector<std::string>
  get_available_modes(const std::string & part) const;

  SYSTEM_MODES_PUBLIC
  virtual
  ~ModeInference() = default;

protected:
  SYSTEM_MODES_PUBLIC
  virtual bool
  matching_parameters(const rclcpp::Parameter &, const rclcpp::Parameter &) const;
  
  SYSTEM_MODES_PUBLIC
  virtual void
  read_modes_from_model(const std::string & model_path);
  
  SYSTEM_MODES_PUBLIC
  virtual void
  add_param_to_mode(ModeBasePtr, const rclcpp::Parameter &);

private:
  ModeHandling * mode_handling_;

  StatesMap nodes_, nodes_target_, nodes_cache_;
  StatesMap systems_, systems_target_, systems_cache_;
  std::map<std::string, ModeMap> modes_;
  ParametersMap parameters_;

  mutable std::shared_timed_mutex
    nodes_mutex_, systems_mutex_,
    modes_mutex_, parts_mutex_,
    param_mutex_;
  mutable std::shared_timed_mutex
    nodes_target_mutex_, systems_target_mutex_;
  mutable std::shared_timed_mutex
    nodes_cache_mutex_, systems_cache_mutex_;
};

}  // namespace system_modes
