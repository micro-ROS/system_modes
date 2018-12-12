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
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <stdexcept>
#include <string>
#include <memory>
#include <vector>

#include "system_modes/mode.hpp"

using std::string;
using std::vector;
using rclcpp::Parameter;

using system_modes::DefaultMode;
using system_modes::DefaultModePtr;

class TestDefaultMode : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  void SetUp()
  {
    default_mode = DefaultModePtr(new DefaultMode());
  }

  void TearDown()
  {
  }

  DefaultModePtr default_mode;
};

/*
   Testing client construction and destruction.
 */
TEST_F(TestDefaultMode, construction_and_destruction) {
  {
    DefaultMode mode;
    EXPECT_EQ("__DEFAULT__", mode.get_name());
  }
}

/*
   Testing add parameter
 */
TEST_F(TestDefaultMode, set_parameter) {
  {
    vector<string> parameter_names({"foo"});

    Parameter param1("foo", "bar");
    vector<Parameter> parameters;
    parameters.push_back(param1);

    default_mode->set_parameter(param1);

    EXPECT_EQ("bar", default_mode->get_parameter("foo").as_string());

    EXPECT_EQ(parameters.size(), default_mode->get_parameters().size());
    EXPECT_EQ(parameter_names, default_mode->get_parameter_names());
  }
}

/*
   Testing add parameters
 */
TEST_F(TestDefaultMode, set_parameters) {
  {
    vector<string> parameter_names({"foo", "fubar"});

    Parameter param1("foo", "bar");
    Parameter param2("fubar", 0.123);

    vector<Parameter> parameters;
    parameters.push_back(param1);
    parameters.push_back(param2);

    default_mode->set_parameters(parameters);

    EXPECT_EQ("bar", default_mode->get_parameter("foo").as_string());
    EXPECT_EQ(0.123, default_mode->get_parameter("fubar").as_double());

    EXPECT_EQ(parameters.size(), default_mode->get_parameters().size());
    EXPECT_EQ(parameter_names, default_mode->get_parameter_names());
  }
}
