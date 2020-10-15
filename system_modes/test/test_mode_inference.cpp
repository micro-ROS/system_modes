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
#include "system_modes/mode_inference.hpp"

using std::string;
using std::vector;
using rclcpp::Parameter;

using system_modes::ModeInference;
using system_modes::StateAndMode;
using system_modes::Deviation;

using lifecycle_msgs::msg::State;

/*
   Testing parsing of mode files
 */
TEST(TestModeFilesParse, parsing) {
  ModeInference * inference;

  EXPECT_THROW(
    inference = new ModeInference(MODE_FILE_WRONG),
    std::out_of_range);

  EXPECT_NO_THROW(inference = new ModeInference(MODE_FILE_CORRECT));
  EXPECT_EQ(3u, inference->get_all_parts().size());
}

TEST(TestModeFilesParse, initialization) {
  ModeInference * inference;

  EXPECT_NO_THROW(inference = new ModeInference(MODE_FILE_CORRECT));

  // parts
  EXPECT_EQ(3u, inference->get_all_parts().size());
  EXPECT_EQ(2u, inference->get_nodes().size());
  EXPECT_EQ(1u, inference->get_systems().size());

  // number of modes
  EXPECT_EQ(3u, inference->get_available_modes("system").size());
  EXPECT_EQ(3u, inference->get_available_modes("part0").size());
  EXPECT_EQ(3u, inference->get_available_modes("part1").size());

  // default modes
  auto mode = inference->get_mode("system", "__DEFAULT__");
  EXPECT_EQ(2u, mode->get_parts().size());
  EXPECT_EQ(State::PRIMARY_STATE_INACTIVE, mode->get_part_mode("part0").state);
  EXPECT_EQ(State::PRIMARY_STATE_ACTIVE, mode->get_part_mode("part1").state);
  mode = inference->get_mode("part0", "__DEFAULT__");
  EXPECT_EQ(2u, mode->get_parameters().size());
  EXPECT_EQ(0.1, mode->get_parameter("foo").as_double());
  EXPECT_EQ("WARN", mode->get_parameter("bar").as_string());
  EXPECT_THROW(mode->get_parameter("baz"), std::out_of_range);
}

TEST(TestModeInference, update_target) {
  ModeInference inference(MODE_FILE_CORRECT);

  StateAndMode active_default(State::PRIMARY_STATE_ACTIVE, "__DEFAULT__");
  StateAndMode inactive(State::PRIMARY_STATE_INACTIVE, "");

  // system
  inference.update_target("system", active_default);
  auto target = inference.get_target("system");
  EXPECT_EQ(active_default.state, target.state);
  EXPECT_EQ(active_default.mode, target.mode);
  inference.update_target("system", inactive);
  target = inference.get_target("system");
  EXPECT_EQ(inactive.state, target.state);
  EXPECT_EQ(inactive.mode, target.mode);

  // node
  inference.update_target("part0", active_default);
  target = inference.get_target("part0");
  EXPECT_EQ(active_default.state, target.state);
  EXPECT_EQ(active_default.mode, target.mode);
  inference.update_target("part0", inactive);
  target = inference.get_target("part0");
  EXPECT_EQ(inactive.state, target.state);
  EXPECT_EQ(inactive.mode, target.mode);
}

TEST(TestModeInference, update_state_and_mode) {
  ModeInference inference(MODE_FILE_CORRECT);

  StateAndMode active_default(State::PRIMARY_STATE_ACTIVE, "__DEFAULT__");
  StateAndMode inactive(State::PRIMARY_STATE_INACTIVE, "");

  inference.update_target("system", active_default);
  EXPECT_THROW(inference.update_state("system", active_default.state), std::out_of_range);
  EXPECT_THROW(inference.update_mode("system", active_default.mode), std::out_of_range);
  inference.update("part0", inactive);
  inference.update_state("part1", active_default.state);
  inference.update_mode("part1", active_default.mode);
  EXPECT_EQ(active_default.state, inference.get_or_infer("system").state);
  EXPECT_EQ(active_default.mode, inference.get_or_infer("system").mode);
}

TEST(TestModeInference, update_parameters) {
  ModeInference inference(MODE_FILE_CORRECT);

  StateAndMode active_default(State::PRIMARY_STATE_ACTIVE, "__DEFAULT__");
  Parameter foo("foo", 0.1);
  Parameter bar("bar", "WARN");

  inference.update_target("part0", active_default);
  EXPECT_THROW(inference.get_or_infer("part0"), std::runtime_error);
  inference.update_param("part0", foo);
  inference.update_param("part0", bar);
  EXPECT_EQ(active_default.state, inference.get_or_infer("part0").state);
  EXPECT_EQ(active_default.mode, inference.get_or_infer("part0").mode);
}

TEST(TestModeInference, inference) {
  ModeInference inference(MODE_FILE_CORRECT);

  // update node modes, test inferred system mode
  StateAndMode active_default(State::PRIMARY_STATE_ACTIVE, "__DEFAULT__");
  StateAndMode inactive(State::PRIMARY_STATE_INACTIVE, "");
  inference.update_target("system", active_default);
  inference.update("part0", inactive);
  inference.update("part1", active_default);

  // System inference
  StateAndMode sm = inference.get_or_infer("system");
  EXPECT_EQ(State::PRIMARY_STATE_ACTIVE, sm.state);
  EXPECT_EQ("__DEFAULT__", sm.mode);

  // Node inference
  Parameter foo("foo", 0.2);
  Parameter bar("bar", "DBG");
  inference.update_param("part1", foo);
  inference.update_param("part1", bar);
  EXPECT_EQ(active_default.state, inference.infer("part1").state);
  EXPECT_EQ("AAA", inference.infer("part1").mode);
  Parameter foo2("foo", 0.1);
  Parameter bar2("bar", "DBG");
  inference.update_state("part0", State::PRIMARY_STATE_ACTIVE);
  inference.update_param("part0", foo2);
  inference.update_param("part0", bar2);
  EXPECT_EQ(active_default.state, inference.infer("part0").state);
  EXPECT_EQ("FOO", inference.infer("part0").mode);

  // System inference
  EXPECT_EQ(State::TRANSITION_STATE_ACTIVATING, inference.infer("system").state);
}

TEST(TestModeInference, infer_transitions) {
  ModeInference inference(MODE_FILE_CORRECT);

  // update node modes, test inferred system mode
  StateAndMode active_default(State::PRIMARY_STATE_ACTIVE, "__DEFAULT__");
  StateAndMode inactive(State::PRIMARY_STATE_INACTIVE, "");
  inference.update_target("system", active_default);
  inference.update("part0", inactive);
  inference.update("part1", active_default);

  // Expect to have one initial transition
  EXPECT_EQ(1u, inference.infer_transitions().size());
  EXPECT_EQ(0u, inference.infer_transitions().size());

  // Expect to have one transition of part0
  Parameter foo("foo", 0.2);
  Parameter bar("bar", "DBG");
  inference.update_param("part1", foo);
  inference.update_param("part1", bar);
  auto transitions = inference.infer_transitions();
  EXPECT_EQ(1u, transitions.size());
  EXPECT_EQ(State::PRIMARY_STATE_ACTIVE, transitions["part1"].first.state);
  EXPECT_EQ("__DEFAULT__", transitions["part1"].first.mode);
  EXPECT_EQ(State::PRIMARY_STATE_ACTIVE, transitions["part1"].second.state);
  EXPECT_EQ("AAA", transitions["part1"].second.mode);

  // Expect to have cleared transitions
  EXPECT_EQ(0u, inference.infer_transitions().size());

  // Expect two have two transitions: part1 and system
  Parameter foo2("foo", 0.1);
  Parameter bar2("bar", "DBG");
  inference.update_state("part0", State::PRIMARY_STATE_ACTIVE);
  inference.update_param("part0", foo2);
  inference.update_param("part0", bar2);
  transitions = inference.infer_transitions();
  EXPECT_EQ(2u, transitions.size());
  EXPECT_EQ(State::PRIMARY_STATE_INACTIVE, transitions["part0"].first.state);
  EXPECT_EQ(State::PRIMARY_STATE_ACTIVE, transitions["part0"].second.state);
  EXPECT_EQ("FOO", transitions["part0"].second.mode);
  EXPECT_EQ(State::PRIMARY_STATE_ACTIVE, transitions["system"].first.state);
  EXPECT_EQ("__DEFAULT__", transitions["system"].first.mode);
  EXPECT_EQ(State::TRANSITION_STATE_ACTIVATING, transitions["system"].second.state);

  // Expect to have cleared transitions
  EXPECT_EQ(0u, inference.infer_transitions().size());
}
