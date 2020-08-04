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
#include "system_modes/mode_monitor.hpp"

#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <lifecycle_msgs/msg/transition_event.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>

#include <rcl_yaml_param_parser/parser.h>
#include <rclcpp/parameter_map.hpp>
#include <rcl_interfaces/srv/list_parameters.hpp>

#include <string>
#include <memory>
#include <utility>
#include <chrono>
#include <ctime>
#include <cstdio>
#include <iostream>
#include <iomanip>

using std::cout;
using std::endl;
using std::setw;
using std::string;
using std::to_string;
using std::make_pair;
using rclcpp::ParameterMap;
using lifecycle_msgs::msg::State;
using lifecycle_msgs::msg::Transition;
using lifecycle_msgs::msg::TransitionEvent;
using system_modes::msg::ModeEvent;

using namespace std::chrono_literals;
using namespace std::literals::string_literals;

namespace system_modes
{

static const bool MONITOR_DEFAULT_VERBOSITY = false;
static const bool MONITOR_DEFAULT_DEBUG = false;
static const unsigned int MONITOR_DEFAULT_RATE_MS = 1000;

static const unsigned int MONITOR_WIDTH_PART = 25;
static const unsigned int MONITOR_WIDTH_STATE = 30;
static const unsigned int MONITOR_WIDTH_MODE = 30;
static const unsigned int MONITOR_WIDTH_TIME = 16;
static const unsigned int MONITOR_WIDTH = MONITOR_WIDTH_PART + MONITOR_WIDTH_STATE +
  MONITOR_WIDTH_MODE + MONITOR_WIDTH_TIME;

static const unsigned int MONITOR_TEXT_COLOR = 37;
static const unsigned int MONITOR_TEXT_WARN_COLOR = 33;
static const unsigned int MONITOR_TEXT_MODE_COLOR = 36;

static const string MONITOR_TEXT_PLAIN = "\033[21;"s + to_string(MONITOR_TEXT_COLOR) + ";40m";
static const string MONITOR_TEXT_BOLD = "\033[01;"s + to_string(MONITOR_TEXT_COLOR) + ";40m";
static const string MONITOR_TEXT_MODE = "\033[21;"s + to_string(MONITOR_TEXT_MODE_COLOR) + ";40m";
static const string MONITOR_TEXT_WARN = "\033[21;"s + to_string(MONITOR_TEXT_WARN_COLOR) + ";40m";

static const string MONITOR_SEPARATOR = MONITOR_TEXT_PLAIN + " | ";
static const string MONITOR_SEPARATOR_BOLD = MONITOR_TEXT_BOLD + " | ";

ModeMonitor::ModeMonitor()
: Node("__mode_monitor"),
  mode_inference_()
{
  declare_parameter(
    "modelfile",
    rclcpp::ParameterValue(std::string("")));
  declare_parameter(
    "rate",
    rclcpp::ParameterValue(static_cast<int>(MONITOR_DEFAULT_RATE_MS)));
  declare_parameter(
    "debug",
    rclcpp::ParameterValue(static_cast<bool>(MONITOR_DEFAULT_DEBUG)));
  declare_parameter(
    "verbose",
    rclcpp::ParameterValue(static_cast<bool>(MONITOR_DEFAULT_VERBOSITY)));

  rate_ = get_parameter("rate").as_int();
  clear_screen_ = !get_parameter("debug").as_bool();
  verbose_ = get_parameter("verbose").as_bool();
  model_path_ = get_parameter("modelfile").as_string();
  if (model_path_.empty()) {
    throw std::invalid_argument("Need path to model file.");
  }
  mode_inference_ = std::make_shared<ModeInference>(model_path_);

  // Start plotting
  timer_ = this->create_wall_timer(std::chrono::milliseconds(rate_), [this]() {this->refresh();});
}

std::shared_ptr<ModeInference>
ModeMonitor::inference()
{
  return this->mode_inference_;
}

void
ModeMonitor::refresh() const
{
  // Clear screen
  if (clear_screen_) {
    cout << "\x1b[1;1f\x1b[J";
  }

  // Header
  cout << this->header();

  // Systems
  cout << MONITOR_HLINE("systems") << endl;
  for (auto & system : this->mode_inference_->get_systems()) {
    unsigned int state_actual = 0, state_infer = 0, state_target = 0;
    string mode_infer, mode_target;

    RCLCPP_DEBUG(
      get_logger(),
      "ModeMonitor::refresh() system %s'",
      system.c_str());

    try {
      auto stateAndMode = mode_inference_->infer(system);
      state_infer = stateAndMode.state;
      mode_infer = stateAndMode.mode;
    } catch (std::out_of_range & ex) {
      RCLCPP_DEBUG(
        get_logger(),
        "ModeMonitor::refresh() Can't do inference for system '%s', due to out_of_range: %s",
        system.c_str(),
        ex.what());
    } catch (std::runtime_error & ex) {
      RCLCPP_DEBUG(
        get_logger(),
        "ModeMonitor::refresh() Can't do inference for system '%s', due to runtime_error: %s",
        system.c_str(),
        ex.what());
    }

    try {
      auto stateAndMode = mode_inference_->get_target(system);
      state_target = stateAndMode.state;
      mode_target = stateAndMode.mode;
    } catch (std::out_of_range & ex) {
      RCLCPP_DEBUG(
        get_logger(),
        "ModeMonitor::refresh() Can't do inference for system '%s', due to out_of_range: %s",
        system.c_str(),
        ex.what());
    } catch (std::runtime_error & ex) {
      RCLCPP_DEBUG(
        get_logger(),
        "ModeMonitor::refresh() Can't do inference for system '%s', due to runtime_error: %s",
        system.c_str(),
        ex.what());
    }

    cout << this->format_line(
      system, state_actual, state_infer, state_target, mode_infer,
      mode_target) << endl;
  }

  // Nodes
  cout << MONITOR_HLINE("nodes") << endl;
  for (auto & node : this->mode_inference_->get_nodes()) {
    unsigned int state_actual = 0, state_infer = 0, state_target = 0;
    string mode_infer, mode_target;

    RCLCPP_DEBUG(
      get_logger(),
      "ModeMonitor::refresh() node %s'",
      node.c_str());

    // state/mode
    try {
      auto stateAndMode = mode_inference_->get(node);
      state_actual = stateAndMode.state;
    } catch (std::out_of_range & ex) {
      RCLCPP_DEBUG(
        get_logger(),
        "ModeMonitor::refresh() Can't get reliable info for part '%s', due to out_of_range: %s",
        node.c_str(),
        ex.what());
    } catch (std::runtime_error & ex) {
      RCLCPP_DEBUG(
        get_logger(),
        "ModeMonitor::refresh() Can't get reliable info for part '%s', due to runtime_error: %s",
        node.c_str(),
        ex.what());
    }

    // state/mode inference
    try {
      auto stateAndMode = mode_inference_->infer(node);
      state_infer = stateAndMode.state;
      mode_infer = stateAndMode.mode;
    } catch (std::out_of_range & ex) {
      RCLCPP_DEBUG(
        get_logger(),
        "ModeMonitor::refresh() Didn't get inference for part '%s', due to out_of_range: %s",
        node.c_str(),
        ex.what());
    } catch (std::runtime_error & ex) {
      RCLCPP_DEBUG(
        get_logger(),
        "ModeMonitor::refresh() Didn't get inference for part '%s', due to runtime_error: %s",
        node.c_str(),
        ex.what());
    }

    // state/mode target
    try {
      auto stateAndMode = mode_inference_->get_target(node);
      state_target = stateAndMode.state;
      mode_target = stateAndMode.mode;
    } catch (std::runtime_error & ex) {
      RCLCPP_DEBUG(
        get_logger(),
        "ModeMonitor::refresh() Can't get target for part '%s', due to runtime_error: %s",
        node.c_str(),
        ex.what());
    } catch (std::out_of_range & ex) {
      RCLCPP_DEBUG(
        get_logger(),
        "ModeMonitor::refresh() Can't get target for part '%s', due to out_of_range: %s",
        node.c_str(),
        ex.what());
    }
    // print
    cout << this->format_line(
      node, state_actual, state_infer, state_target, mode_infer,
      mode_target);
    auto mode = mode_inference_->get_mode(node, mode_infer);
    if (verbose_ && mode) {
      cout << MONITOR_TEXT_MODE << mode->print();
    }
    cout << endl;
  }

  // Footer
  cout << MONITOR_TEXT_PLAIN << endl << string(MONITOR_WIDTH - 2, '-') << endl << " (*) = inferred";
}

string
ModeMonitor::header() const
{
  auto p = std::chrono::system_clock::now();
  std::time_t t = std::chrono::system_clock::to_time_t(p);
  std::ostringstream os(std::ostringstream::out);
  os.precision(2);

  os << MONITOR_TEXT_BOLD << " System Modes Monitor - " << std::ctime(&t);
  os << MONITOR_TEXT_PLAIN << " Model: " << model_path_;
  os << endl << endl;

  os << MONITOR_TEXT_BOLD << setw(MONITOR_WIDTH_PART) << "part" << MONITOR_SEPARATOR_BOLD;
  os << MONITOR_TEXT_BOLD << setw(MONITOR_WIDTH_STATE + 3) << "state" << MONITOR_SEPARATOR_BOLD;
  os << MONITOR_TEXT_BOLD << setw(MONITOR_WIDTH_MODE + 3) << "mode" << MONITOR_SEPARATOR_BOLD;
  os << endl;

  os << MONITOR_TEXT_PLAIN << setw(MONITOR_WIDTH_PART) << "" << MONITOR_SEPARATOR_BOLD;
  os << MONITOR_TEXT_PLAIN << setw(MONITOR_WIDTH_STATE / 2) << "target" << MONITOR_SEPARATOR;
  os << MONITOR_TEXT_PLAIN << setw(MONITOR_WIDTH_STATE / 2) << "actual" << MONITOR_SEPARATOR_BOLD;
  os << MONITOR_TEXT_PLAIN << setw(MONITOR_WIDTH_MODE / 2) << "target" << MONITOR_SEPARATOR;
  os << MONITOR_TEXT_PLAIN << setw(MONITOR_WIDTH_MODE / 2) << "actual" << MONITOR_SEPARATOR_BOLD;
  os << endl;

  return os.str();
}

string
ModeMonitor::format_line(
  const string & part,
  unsigned int state_actual, unsigned int state_infer, unsigned int state_target,
  const string & mode_infer, const string & mode_target) const
{
  unsigned int state = State::PRIMARY_STATE_UNKNOWN;
  string mode = mode_infer;

  string s_display = " ", s_target = " ";
  string m_display = " ", m_target = " ";
  string s_style = MONITOR_TEXT_PLAIN, m_style = MONITOR_TEXT_PLAIN;

  // state display
  if (state_actual != State::PRIMARY_STATE_UNKNOWN) {
    s_display = state_label_(state_actual);
  } else if (state_infer != State::PRIMARY_STATE_UNKNOWN) {
    state = state_infer;
    s_display = state_label_(state_infer) + "(*)";
  }
  if (state != State::PRIMARY_STATE_UNKNOWN && state_target != State::PRIMARY_STATE_UNKNOWN &&
    state != state_target)
  {
    s_style = MONITOR_TEXT_WARN;
  }

  // state target
  if (state_target != State::PRIMARY_STATE_UNKNOWN) {
    s_target = state_label_(state_target);
  }

  // mode display
  if (!mode_infer.empty()) {
    m_display = mode_infer + "(*)";
  }
  if (!mode_infer.empty() && mode_infer.compare(mode_target) != 0) {
    m_style = MONITOR_TEXT_WARN;
  }

  // state target
  if (!mode_target.empty()) {
    m_target = mode_target;
  }

  std::ostringstream os(std::ostringstream::out);
  os.precision(2);
  os << MONITOR_TEXT_PLAIN << setw(MONITOR_WIDTH_PART) << part << MONITOR_SEPARATOR_BOLD;
  os << MONITOR_TEXT_PLAIN << setw(MONITOR_WIDTH_STATE / 2) << s_target << MONITOR_SEPARATOR;
  os << s_style << setw(MONITOR_WIDTH_STATE / 2) << s_display << MONITOR_SEPARATOR;
  os << MONITOR_TEXT_PLAIN << setw(MONITOR_WIDTH_MODE / 2) << m_target << MONITOR_SEPARATOR;
  os << m_style << setw(MONITOR_WIDTH_MODE / 2) << m_display << MONITOR_SEPARATOR;

  return os.str();
}

inline const string ModeMonitor::MONITOR_HLINE(const string & label) const
{
  std::ostringstream os(std::ostringstream::out);
  os << MONITOR_TEXT_BOLD << "- " << label << " " <<
    string(MONITOR_WIDTH - label.length() - 5, '-');
  return os.str();
}

}  // namespace system_modes
