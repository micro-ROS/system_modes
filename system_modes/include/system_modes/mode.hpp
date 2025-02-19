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

#include <map>
#include <mutex>
#include <string>
#include <vector>
#include <memory>
#include <utility>
#include <iostream>

#include <rclcpp/macros.hpp>
#include <rclcpp/node_interfaces/node_parameters.hpp>
#include <rclcpp/parameter_map.hpp>

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
  // cppcheck-suppress unknownMacro
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

/**
 * Represents a default system mode.
 *
 * For default modes of nodes it holds the according parametrization, for
 * default modes of systems, it holds the according states and modes of its
 * system parts.
 */
class DefaultMode : public ModeBase
{
public:
  DefaultMode();
  explicit DefaultMode(const std::string & mode_name) = delete;
  // cppcheck-suppress unknownMacro
  RCLCPP_DISABLE_COPY(DefaultMode)

  virtual void set_parameter(const rclcpp::Parameter & parameter);
  virtual void set_parameters(const std::vector<rclcpp::Parameter> & parameters);

  virtual void set_part_mode(
    const std::string & part,
    const StateAndMode stateAndMode);
};

/**
 * Represents a (non-default) system mode.
 *
 * For default modes of nodes it holds the according parametrization, for
 * default modes of systems, it holds the according states and modes of its
 * system parts.
 */
class Mode : public ModeBase
{
public:
  explicit Mode(const std::string & mode_name) = delete;
  Mode(const std::string & mode_name, const DefaultModePtr default_mode);
  // cppcheck-suppress unknownMacro
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

}  // namespace system_modes
