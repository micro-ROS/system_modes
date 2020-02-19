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

#include "system_modes/mode_impl.hpp"

#include <rclcpp/node_interfaces/node_parameters.hpp>
#include <rclcpp/parameter_map.hpp>

#include <map>
#include <mutex>
#include <string>
#include <vector>
#include <memory>
#include <utility>
#include <iostream>

namespace system_modes
{

class ModeBase;
class DefaultMode;
class Mode;

using ModeBasePtr = std::shared_ptr<ModeBase>;
using ModeConstPtr = std::shared_ptr<const ModeBase>;
using ModePtr = std::shared_ptr<Mode>;
using DefaultModePtr = std::shared_ptr<DefaultMode>;

using ModeMap = std::map<std::string, ModeConstPtr>;

class ModeBase
{
public:
  explicit ModeBase(const std::string & mode_name);
  virtual ~ModeBase() = default;
  RCLCPP_DISABLE_COPY(ModeBase)

  std::string get_name() const;

  virtual void set_parameter(const rclcpp::Parameter & parameter) = 0;
  virtual void set_parameters(const std::vector<rclcpp::Parameter> & parameters) = 0;
  virtual void set_part_mode(
    const std::string & part,
    const StateAndMode stateAndMode) = 0;

  virtual rclcpp::Parameter get_parameter(const std::string & param_name) const;
  virtual std::vector<std::string> get_parameter_names() const;
  virtual const std::vector<rclcpp::Parameter> get_parameters() const;

  virtual const std::vector<std::string> get_parts() const;
  virtual const StateAndMode get_part_mode(const std::string & part) const;

  virtual std::string print() const;

protected:
  ModeImpl mode_impl_;
};

class DefaultMode : public ModeBase
{
public:
  DefaultMode();
  explicit DefaultMode(const std::string & mode_name) = delete;
  RCLCPP_DISABLE_COPY(DefaultMode)

  virtual void set_parameter(const rclcpp::Parameter & parameter);
  virtual void set_parameters(const std::vector<rclcpp::Parameter> & parameters);

  virtual void set_part_mode(
    const std::string & part,
    const StateAndMode stateAndMode);
};

class Mode : public ModeBase
{
public:
  explicit Mode(const std::string & mode_name) = delete;
  Mode(const std::string & mode_name, const DefaultModePtr default_mode);
  RCLCPP_DISABLE_COPY(Mode)

  virtual ~Mode() = default;

  virtual void set_parameter(const rclcpp::Parameter & parameter);
  virtual void set_parameters(const std::vector<rclcpp::Parameter> & parameters);

  virtual void set_part_mode(
    const std::string & part,
    const StateAndMode stateAndMode);
};

inline std::ostream & operator<<(std::ostream & os, const Mode & mode)
{
  os.precision(3);
  os << mode.print();
  return os;
}

unsigned int state_id_(const std::string & state_label);
const std::string state_label_(unsigned int state_id);

unsigned int transition_id_(const std::string & transition_label);
const std::string transition_label_(unsigned int transition_id);

unsigned int goal_state_(unsigned int transition_id);

}  // namespace system_modes
