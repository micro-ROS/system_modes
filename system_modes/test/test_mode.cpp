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

using system_modes::Mode;
using system_modes::DefaultMode;
using system_modes::DefaultModePtr;

class TestMode : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  void SetUp()
  {
    default_mode = DefaultModePtr(new DefaultMode());

    Parameter param1("foo", "bar");
    Parameter param2("fubar", 0.123);

    vector<Parameter> parameters;
    parameters.push_back(param1);
    parameters.push_back(param2);

    default_mode->set_parameters(parameters);
  }

  void TearDown()
  {
  }

  DefaultModePtr default_mode;
};

/*
   Testing client construction and destruction.
 */
TEST_F(TestMode, construction_and_destruction) {
  {
    vector<string> parameter_names({"foo", "fubar"});
    DefaultModePtr default_mode_null;

    Mode * mode;

    EXPECT_THROW(
      mode = new Mode("MODE", default_mode_null),
      std::runtime_error);
    EXPECT_NO_THROW(mode = new Mode("MODE", default_mode));

    EXPECT_EQ(parameter_names, mode->get_parameter_names());
    EXPECT_EQ("bar", mode->get_parameter("foo").as_string());
    EXPECT_EQ(0.123, mode->get_parameter("fubar").as_double());
  }
}

/*
   Testing set parameter
 */
TEST_F(TestMode, set_parameter) {
  {
    vector<string> parameter_names({"foo", "fubar"});

    Parameter param1("foo", "baz");
    Parameter param2("fubar", 0.234);
    Parameter param3("fuuubar", true);

    Mode mode("MODE", default_mode);

    mode.set_parameter(param1);
    mode.set_parameter(param2);
    EXPECT_EQ("baz", mode.get_parameter("foo").as_string());
    EXPECT_EQ(0.234, mode.get_parameter("fubar").as_double());

    EXPECT_THROW(mode.set_parameter(param3), std::out_of_range);

    EXPECT_EQ(parameter_names, mode.get_parameter_names());
  }
}

/*
   Testing set parameters
 */
TEST_F(TestMode, set_parameters) {
  {
    vector<string> parameter_names({"foo", "fubar"});

    Parameter param1("foo", "baz");
    Parameter param2("fubar", 0.234);

    vector<Parameter> parameters;
    parameters.push_back(param1);
    parameters.push_back(param2);

    Mode mode("MODE", default_mode);
    EXPECT_NO_THROW(mode.set_parameters(parameters));

    EXPECT_EQ("baz", mode.get_parameter("foo").as_string());
    EXPECT_EQ(0.234, mode.get_parameter("fubar").as_double());

    EXPECT_EQ(parameters.size(), mode.get_parameters().size());
    EXPECT_EQ(parameter_names, mode.get_parameter_names());

    Parameter param3("fuuubar", true);
    parameters.push_back(param3);
    EXPECT_THROW(mode.set_parameters(parameters), std::out_of_range);

    EXPECT_EQ((unsigned int) 2, mode.get_parameters().size());
    EXPECT_EQ(parameter_names, mode.get_parameter_names());
  }
}
