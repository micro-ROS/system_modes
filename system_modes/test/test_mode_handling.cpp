// Copyright (c) 2018 - for information on the respective copyright owner
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
#include <lifecycle_msgs/msg/state.hpp>

#include <gtest/gtest.h>

#include <stdexcept>
#include <string>
#include <memory>
#include <vector>

#include "system_modes/modefiles.h"
#include "system_modes/mode_handling.hpp"

using std::string;
using std::vector;
using rclcpp::Parameter;

using system_modes::ModeHandling;
using system_modes::StateAndMode;

using lifecycle_msgs::msg::State;

/*
   Testing parsing of mode files
 */
TEST(TestModeFilesParse, constructor) {
  ModeHandling * handling;

  EXPECT_NO_THROW(handling = new ModeHandling(MODE_FILE_CORRECT));
  EXPECT_NO_THROW(handling = new ModeHandling(MODE_FILE_RULES));

  EXPECT_THROW(
    handling = new ModeHandling("incorrect path"),
    std::runtime_error);

  (void) handling;
}

/*
   Testing parsing of mode files
 */
TEST(TestModeFilesParse, parse_rules) {
  ModeHandling * handling = new ModeHandling(MODE_FILE_RULES);

  auto target = StateAndMode(State::PRIMARY_STATE_ACTIVE, "");
  EXPECT_EQ(0u, handling->get_rules_for("system", target).size());

  target = StateAndMode(State::PRIMARY_STATE_ACTIVE, "AA");
  EXPECT_EQ(2u, handling->get_rules_for("system", target).size());

  target = StateAndMode(State::PRIMARY_STATE_ACTIVE, "BB");
  EXPECT_EQ(2u, handling->get_rules_for("system", target).size());
}

/*
   Testing parsing of mode files
 */
TEST(TestModeFilesParse, test_rules) {
  ModeHandling * handling = new ModeHandling(MODE_FILE_RULES);

  auto rules = handling->get_rules_for("system", StateAndMode(State::PRIMARY_STATE_ACTIVE, "AA"));

  EXPECT_EQ("degrade_from_AA", rules[0].name);
  EXPECT_EQ("system", rules[0].system);
  EXPECT_EQ(StateAndMode(State::PRIMARY_STATE_ACTIVE, "AA"), rules[0].system_target);
  EXPECT_EQ("part0", rules[0].part);
  EXPECT_EQ(StateAndMode(State::PRIMARY_STATE_INACTIVE), rules[0].part_actual);
  EXPECT_EQ(StateAndMode(State::PRIMARY_STATE_ACTIVE), rules[0].new_system_target);

  EXPECT_EQ("inactive_from_AA", rules[1].name);
  EXPECT_EQ("system", rules[1].system);
  EXPECT_EQ(StateAndMode(State::PRIMARY_STATE_ACTIVE, "AA"), rules[1].system_target);
  EXPECT_EQ("part1", rules[1].part);
  EXPECT_EQ(StateAndMode(State::PRIMARY_STATE_INACTIVE), rules[1].part_actual);
  EXPECT_EQ(StateAndMode(State::PRIMARY_STATE_INACTIVE), rules[1].new_system_target);

  rules = handling->get_rules_for("system", StateAndMode(State::PRIMARY_STATE_ACTIVE, "BB"));

  EXPECT_EQ("degrade_from_BB", rules[0].name);
  EXPECT_EQ("system", rules[0].system);
  EXPECT_EQ(StateAndMode(State::PRIMARY_STATE_ACTIVE, "BB"), rules[0].system_target);
  EXPECT_EQ("part0", rules[0].part);
  EXPECT_EQ(StateAndMode(State::PRIMARY_STATE_INACTIVE), rules[0].part_actual);
  EXPECT_EQ(StateAndMode(State::PRIMARY_STATE_ACTIVE), rules[0].new_system_target);

  EXPECT_EQ("inactive_from_BB", rules[1].name);
  EXPECT_EQ("system", rules[1].system);
  EXPECT_EQ(StateAndMode(State::PRIMARY_STATE_ACTIVE, "BB"), rules[1].system_target);
  EXPECT_EQ("part1", rules[1].part);
  EXPECT_EQ(StateAndMode(State::PRIMARY_STATE_INACTIVE), rules[1].part_actual);
  EXPECT_EQ(StateAndMode(State::PRIMARY_STATE_INACTIVE), rules[1].new_system_target);
}
