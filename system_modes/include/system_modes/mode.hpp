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

#include <rclcpp/macros.hpp>
#include <rclcpp/node_interfaces/node_parameters.hpp>
#include <rclcpp/parameter_map.hpp>

#include <map>
#include <mutex>
#include <string>
#include <vector>
#include <memory>
#include <utility>
#include <iostream>

#include "system_modes/visibility_control.hpp"

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
  SYSTEM_MODES_PUBLIC
  explicit ModeBase(const std::string & mode_name);

  SYSTEM_MODES_PUBLIC
  virtual ~ModeBase() = default;

  // cppcheck-suppress unknownMacro
  SYSTEM_MODES_PUBLIC
    RCLCPP_DISABLE_COPY(ModeBase)

  SYSTEM_MODES_PUBLIC
  std::string
  get_name() const;

  SYSTEM_MODES_PUBLIC
  virtual void
  set_parameter(const rclcpp::Parameter & parameter) = 0;

  SYSTEM_MODES_PUBLIC
  virtual void
  set_parameters(const std::vector<rclcpp::Parameter> & parameters) = 0;

  SYSTEM_MODES_PUBLIC
  virtual void
  set_part_mode(
    const std::string & part,
    const StateAndMode stateAndMode) = 0;

  SYSTEM_MODES_PUBLIC
  virtual rclcpp::Parameter
  get_parameter(const std::string & param_name) const;

  SYSTEM_MODES_PUBLIC
  virtual std::vector<std::string>
  get_parameter_names() const;

  SYSTEM_MODES_PUBLIC
  virtual const std::vector<rclcpp::Parameter>
  get_parameters() const;

  SYSTEM_MODES_PUBLIC
  virtual const std::vector<std::string>
  get_parts() const;

  SYSTEM_MODES_PUBLIC
  virtual const StateAndMode
  get_part_mode(const std::string & part) const;

  SYSTEM_MODES_PUBLIC
  virtual std::string
  print() const;

protected:
  ModeImpl mode_impl_;
};

class DefaultMode : public ModeBase
{
public:
  SYSTEM_MODES_PUBLIC
  DefaultMode();

  SYSTEM_MODES_PUBLIC
  explicit DefaultMode(const std::string & mode_name) = delete;
  // cppcheck-suppress unknownMacro
  RCLCPP_DISABLE_COPY(DefaultMode)

  SYSTEM_MODES_PUBLIC
  virtual void
  set_parameter(const rclcpp::Parameter & parameter);

  SYSTEM_MODES_PUBLIC
  virtual void
  set_parameters(const std::vector<rclcpp::Parameter> & parameters);

  SYSTEM_MODES_PUBLIC
  virtual void
  set_part_mode(
    const std::string & part,
    const StateAndMode stateAndMode);
};

class Mode : public ModeBase
{
public:
  SYSTEM_MODES_PUBLIC
  explicit Mode(const std::string & mode_name) = delete;

  SYSTEM_MODES_PUBLIC
  Mode(const std::string & mode_name, const DefaultModePtr default_mode);
  // cppcheck-suppress unknownMacro
  RCLCPP_DISABLE_COPY(Mode)

  SYSTEM_MODES_PUBLIC
  virtual
  ~Mode() = default;

  SYSTEM_MODES_PUBLIC
  virtual void
  set_parameter(const rclcpp::Parameter & parameter);

  SYSTEM_MODES_PUBLIC
  virtual void
  set_parameters(const std::vector<rclcpp::Parameter> & parameters);

  SYSTEM_MODES_PUBLIC
  virtual void
  set_part_mode(
    const std::string & part,
    const StateAndMode stateAndMode);
};

SYSTEM_MODES_PUBLIC
inline std::ostream &
operator<<(std::ostream & os, const Mode & mode)
{
  os.precision(3);
  os << mode.print();
  return os;
}

}  // namespace system_modes
