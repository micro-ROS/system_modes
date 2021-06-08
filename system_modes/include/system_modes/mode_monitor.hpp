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

#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <map>
#include <string>
#include <memory>
#include <vector>
#include <utility>

#include "system_modes/mode.hpp"
#include "system_modes/mode_inference.hpp"
#include "system_modes/visibility_control.hpp"

#include "system_modes_msgs/srv/change_mode.hpp"
#include "system_modes_msgs/srv/get_mode.hpp"
#include "system_modes_msgs/srv/get_available_modes.hpp"
#include "system_modes_msgs/msg/mode_event.hpp"

namespace system_modes
{

class ModeMonitor : public rclcpp::Node
{
public:
  SYSTEM_MODES_PUBLIC
  ModeMonitor();

  SYSTEM_MODES_PUBLIC
  ModeMonitor(const ModeMonitor &) = delete;

  SYSTEM_MODES_PUBLIC
  virtual
  ~ModeMonitor();

  SYSTEM_MODES_PUBLIC
  std::shared_ptr<ModeInference>
  inference();

protected:
  SYSTEM_MODES_PUBLIC
  virtual void
  refresh() const;

  SYSTEM_MODES_PUBLIC
  virtual std::string
  header() const;

  SYSTEM_MODES_PUBLIC
  virtual std::string
  format_line(
    const std::string &,
    unsigned int, unsigned int, unsigned int,
    const std::string &, const std::string &) const;

  SYSTEM_MODES_LOCAL
  inline const
  std::string MONITOR_HLINE(const std::string & label) const;

  std::shared_ptr<ModeInference> mode_inference_;
  std::string model_path_;

private:
  rclcpp::TimerBase::SharedPtr timer_;

  int64_t rate_;
  bool clear_screen_, verbose_;
};

}  // namespace system_modes
