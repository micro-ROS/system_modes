// Copyright (c) 2021 - for information on the respective copyright owner
// see the NOTICE file and/or the repository https://github.com/microros/system_modes
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//   http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <stdexcept>

#include "system_modes/modefiles.h"
#include "system_modes/mode_monitor.hpp"

using system_modes::ModeMonitor;

TEST(TestModeMonitor, initialization) {
  ModeMonitor * monitor;

  EXPECT_THROW(monitor = new ModeMonitor(), std::invalid_argument);

  (void) monitor;
}
