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

#include "modefiles.h"
#include "system_modes/mode_inference.hpp"

using std::string;
using std::vector;
using rclcpp::Parameter;

using namespace system_modes;

/*
   Testing parsing of mode files
 */
TEST(TestModeFilesParse, wrong) {
    ModeInference * inference;

    EXPECT_THROW(
        inference = new ModeInference(MODE_FILE_WRONG),
        std::out_of_range);
}

TEST(TestModeFilesParse, correct) {
    ModeInference * inference;

    EXPECT_NO_THROW(inference = new ModeInference(MODE_FILE_CORRECT));

    EXPECT_EQ(3, inference->get_all_parts().size());
    EXPECT_EQ(2, inference->get_nodes().size());
    EXPECT_EQ(1, inference->get_systems().size());
}
