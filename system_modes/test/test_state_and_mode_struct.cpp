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

#include <gtest/gtest.h>
#include <lifecycle_msgs/msg/state.hpp>

#include <string>
#include <iostream>

#include "system_modes/mode.hpp"

using std::string;
using namespace std::string_literals;

using system_modes::StateAndMode;
using lifecycle_msgs::msg::State;

class TestStateAndMode : public ::testing::Test
{
protected:
  void SetUp()
  {
    inactive.state = State::PRIMARY_STATE_INACTIVE;

    active.state = State::PRIMARY_STATE_ACTIVE;

    active_default.state = State::PRIMARY_STATE_ACTIVE;
    active_default.mode = "__DEFAULT__";

    active_foo.state = State::PRIMARY_STATE_ACTIVE;
    active_foo.mode = "FOO";
  }

  void TearDown()
  {
  }

  StateAndMode inactive, active, active_default, active_foo;
};

TEST_F(TestStateAndMode, comparison) {
  {
    EXPECT_EQ(inactive, inactive);
    EXPECT_EQ(active, active);
    EXPECT_EQ(active, active_default);

    EXPECT_NE(active, inactive);
    EXPECT_NE(active, active_foo);
    EXPECT_NE(active_default, active_foo);
    EXPECT_NE(active_default, inactive);

    active_foo.state = State::PRIMARY_STATE_INACTIVE;
    EXPECT_EQ(inactive, active_foo);
  }
}

TEST_F(TestStateAndMode, string_getter) {
  {
    EXPECT_EQ("inactive"s, inactive.as_string());
    EXPECT_EQ("active"s, active.as_string());
    EXPECT_EQ("active.__DEFAULT__"s, active_default.as_string());
    EXPECT_EQ("active.FOO"s, active_foo.as_string());
    active_foo.state = State::PRIMARY_STATE_INACTIVE;
    EXPECT_EQ("inactive"s, active_foo.as_string());
  }
}

TEST_F(TestStateAndMode, string_setter) {
  {
    StateAndMode copy;

    copy.from_string("active");
    EXPECT_EQ(active, copy);

    copy.from_string("active.__DEFAULT__");
    EXPECT_EQ(active_default, copy);

    copy.from_string("active.FOO");
    EXPECT_EQ(active_foo, copy);

    copy.from_string("inactive");
    EXPECT_EQ(inactive, copy);

    copy.from_string(active.as_string());
    EXPECT_EQ(active, copy);

    copy.from_string(active_foo.as_string());
    EXPECT_EQ(active_foo, copy);

    copy.from_string(inactive.as_string());
    EXPECT_EQ(inactive, copy);
  }
}
