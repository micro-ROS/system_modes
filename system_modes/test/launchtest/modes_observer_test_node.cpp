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
#include <rcl_interfaces/msg/parameter_event.hpp>
#include <rcl_interfaces/msg/parameter_type.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition_event.hpp>

#include <cassert>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <utility>
#include <iostream>

#include "system_modes/mode_observer.hpp"

using std::cerr;
using std::cout;
using std::string;
using std::vector;
using std::function;
using std::make_pair;
using std::shared_ptr;

using system_modes::ModeObserver;
using system_modes::DEFAULT_MODE;
using system_modes::StateAndMode;
using system_modes_msgs::msg::ModeEvent;

using lifecycle_msgs::msg::State;
using lifecycle_msgs::msg::TransitionEvent;

using rcl_interfaces::msg::ParameterType;
using rcl_interfaces::msg::ParameterEvent;

using namespace std::chrono_literals;

class ModeObserverNode : public rclcpp::Node
{
public:
  ModeObserverNode()
  : Node("testobservernode")
  {
  }
  void start_observing()
  {
    observer = std::make_shared<ModeObserver>(shared_from_this());
    observer->observe("sys");
    observer->observe("A");
    observer->observe("B");

    periodic_timer = this->create_wall_timer(
      500ms,
      [this]() {
        std::cout << "cached:sys:" << observer->get("sys").as_string() << std::endl;
        std::cout << "cached:A:" << observer->get("A").as_string() << std::endl;
        std::cout << "cached:B:" << observer->get("B").as_string() << std::endl;
      });

    auto sm = observer->get("foo");
    assert(sm.state == 0);
    assert(sm.mode.compare("") == 0);
  }

private:
  std::shared_ptr<ModeObserver> observer;
  rclcpp::TimerBase::SharedPtr periodic_timer;
};

int main(int argc, char * argv[])
{
  using namespace std::placeholders;

  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  auto observer = std::make_shared<ModeObserverNode>();
  observer->start_observing();

  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(observer);
  exe.spin();

  rclcpp::shutdown();

  return 0;
}
