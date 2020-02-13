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
#include "system_modes/mode_impl.hpp"

#include <string>
#include <vector>
#include <utility>
#include <exception>

using std::pair;
using std::mutex;
using std::string;
using std::vector;
using std::lock_guard;
using std::out_of_range;
using rclcpp::Parameter;

namespace system_modes
{

ModeImpl::ModeImpl(const string & mode_name)
: name_(mode_name), param_(), part_modes_(), mutex_()
{
  if (mode_name.empty()) {
    throw std::invalid_argument("Mode name can't be empty.");
  }
}

string
ModeImpl::get_name() const
{
  return this->name_;
}

Parameter
ModeImpl::get_parameter(const string & param_name) const
{
  Parameter parameter;
  if (this->get_parameter(param_name, parameter)) {
    return parameter;
  } else {
    throw out_of_range("Parameter '" + param_name + "' not set");
  }
}

bool
ModeImpl::get_parameter(const string & param_name, Parameter & parameter) const
{
  lock_guard<mutex> lock(this->mutex_);

  if (this->param_.count(param_name)) {
    parameter = this->param_.at(param_name);
    return true;
  } else {
    return false;
  }
}

vector<string>
ModeImpl::get_parameter_names() const
{
  lock_guard<mutex> lock(mutex_);

  vector<string> results;
  for (auto p : this->param_) {
    results.push_back(p.first);
  }
  return results;
}

const vector<Parameter>
ModeImpl::get_parameters() const
{
  vector<Parameter> params;
  for (auto it = this->param_.begin(); it != this->param_.end(); ++it) {
    params.push_back(it->second);
  }
  return params;
}

void
ModeImpl::add_parameter(const Parameter & parameter)
{
  lock_guard<mutex> lock(this->mutex_);

  this->param_.emplace(parameter.get_name(), parameter);
}

void
ModeImpl::add_parameters(const vector<Parameter> & parameters)
{
  lock_guard<mutex> lock(this->mutex_);

  for (auto p : parameters) {
    this->param_[p.get_name()] = p;
  }
}

void
ModeImpl::set_parameter(const Parameter & parameter)
{
  std::string param_name = parameter.get_name();
  std::size_t foundr = parameter.get_name().rfind(".");
  if (foundr != std::string::npos) {
    param_name = parameter.get_name().substr(foundr + 1);
  }

  if (this->param_.find(param_name) == this->param_.end()) {
    throw out_of_range(
            "Parameter '" + param_name + "' not available in mode '" +
            this->name_ + "', has to be present in default mode.");
  }

  this->param_[param_name] = parameter;
}

void
ModeImpl::set_parameters(const vector<Parameter> & parameters)
{
  for (auto p : parameters) {
    this->set_parameter(p);
  }
}

void
ModeImpl::add_part_mode(
  const std::string & part,
  const std::pair<unsigned int, std::string> stateAndMode)
{
  this->part_modes_[part] = stateAndMode;
}

void
ModeImpl::set_part_mode(
  const std::string & part,
  const std::pair<unsigned int, std::string> stateAndMode)
{
  if (this->part_modes_.find(part) == this->part_modes_.end()) {
    throw out_of_range(
            "Part mode for part '" + part + "' not available in mode '" +
            this->name_ + "', has to be present in default mode.");
  }

  this->add_part_mode(part, stateAndMode);
}

const std::vector<std::string>
ModeImpl::get_parts() const
{
  vector<string> results;
  for (auto p : this->part_modes_) {
    results.push_back(p.first);
  }
  return results;
}

const std::pair<unsigned int, std::string>
ModeImpl::get_part_mode(const std::string & part) const
{
  if (this->part_modes_.count(part)) {
    return this->part_modes_.at(part);
  } else {
    return std::make_pair(0, "");
  }
}

}  // namespace system_modes
