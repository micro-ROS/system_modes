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

#include <memory>
#include <stdexcept>

#include "system_modes/modefiles.h"
#include "system_modes/mode_observer.hpp"

using system_modes::ModeObserver;
using system_modes::StateAndMode;

class TestModeObserver : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  void SetUp()
  {
    rclcpp::executors::SingleThreadedExecutor exe;
    my_node = std::make_shared<rclcpp::Node>("testobserver");
    observer = new ModeObserver(my_node);
  }

  void TearDown()
  {
    delete observer;
  }

  ModeObserver * observer;
  std::shared_ptr<rclcpp::Node> my_node;
};

TEST_F(TestModeObserver, initialization) {
  auto my_node = std::make_shared<rclcpp::Node>("testobserver");

  ModeObserver * my_observer;
  EXPECT_NO_THROW(my_observer = new ModeObserver(my_node));

  (void) my_observer;
}

TEST_F(TestModeObserver, start_stop_observing) {
  EXPECT_NO_THROW(observer->observe("foo"));

  EXPECT_NO_THROW(observer->stop_observing("foo"));
  EXPECT_NO_THROW(observer->stop_observing("bar"));
}

TEST_F(TestModeObserver, observing) {
  EXPECT_NO_THROW(observer->observe("foo"));

  StateAndMode sm;
  EXPECT_NO_THROW(sm = observer->get("foo"));
  EXPECT_EQ(0u, sm.state);
  EXPECT_STREQ("", sm.mode.c_str());
}
