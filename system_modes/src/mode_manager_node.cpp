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

#include <boost/program_options.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <utility>
#include <iostream>

#include "system_modes/mode_manager.hpp"

using std::cerr;
using std::cout;
using std::string;
using std::vector;
using std::function;
using std::make_pair;
using std::shared_ptr;

using system_modes::ModeManager;
using system_modes::DEFAULT_MODE;
using system_modes::msg::ModeEvent;

using lifecycle_msgs::msg::State;
using lifecycle_msgs::msg::TransitionEvent;

using boost::program_options::value;
using boost::program_options::variables_map;
using boost::program_options::command_line_parser;
using boost::program_options::options_description;
using boost::program_options::positional_options_description;

using rcl_interfaces::msg::ParameterType;
using rcl_interfaces::msg::ParameterEvent;

string modelfile, loglevel;
options_description options("Allowed options");

shared_ptr<ModeManager> manager;

bool parseOptions(int argc, char * argv[])
{
  options.add_options()("help", "Help message and options")(
    "modelfile", value<string>(&modelfile),
    "Path to yaml model file")(
    "__log_level", value<string>(&loglevel),
    "ROS2 log level")(
    "ros-args", value<vector<string>>()->multitoken(), "ROS args")(
    "params-file", value<vector<string>>()->multitoken(), "ROS params file");

  positional_options_description positional_options;
  positional_options.add("modelfile", 1);

  variables_map map;
  store(
    command_line_parser(argc, argv)
    .options(options)
    .positional(positional_options)
    .run(), map);
  notify(map);

  if (map.count("help")) {
    return true;
  }
  return false;
}

void transition_callback(
  const TransitionEvent::SharedPtr msg,
  const string & node_name)
{
  manager->inference()->update_state(node_name, msg->goal_state.id);
}

void mode_change_callback(
  const ModeEvent::SharedPtr msg,
  const string & node_name)
{
  manager->inference()->update_state(node_name, State::PRIMARY_STATE_ACTIVE);
  manager->inference()->update_mode(node_name, msg->goal_mode.label.c_str());
}

void transition_request_callback(
  const TransitionEvent::SharedPtr msg,
  const string & node_name)
{
  if (msg->goal_state.id != State::PRIMARY_STATE_ACTIVE) {
    manager->inference()->update_target(
      node_name,
      make_pair(msg->goal_state.id, ""));
  } else {
    manager->inference()->update_target(
      node_name,
      make_pair(msg->goal_state.id, DEFAULT_MODE));
  }
}

void mode_request_callback(
  const ModeEvent::SharedPtr msg,
  const string & node_name)
{
  manager->inference()->update_target(
    node_name,
    make_pair(State::PRIMARY_STATE_ACTIVE, msg->goal_mode.label.c_str()));
}

void
parameter_event_callback(const ParameterEvent::SharedPtr event)
{
  for (auto p : event->new_parameters) {
    auto param = rclcpp::Parameter::from_parameter_msg(p);
    manager->inference()->update_param(event->node, param);
  }
  for (auto p : event->changed_parameters) {
    auto param = rclcpp::Parameter::from_parameter_msg(p);
    manager->inference()->update_param(event->node, param);
  }
  for (auto p : event->deleted_parameters) {
    auto param = rclcpp::Parameter::from_parameter_msg(p);
    manager->inference()->update_param(event->node, param);
  }
}

int main(int argc, char * argv[])
{
  using namespace std::placeholders;

  // Handle commandline arguments.
  try {
    if (parseOptions(argc, argv)) {
      cout << "Usage: mode_manager MODELFILE" << std::endl;
      cout << options;
      cout << "Or specify the MODELFILE by ROS parameter 'modelfile'." << std::endl << std::endl;
      return EXIT_SUCCESS;
    }
  } catch (const std::exception & e) {
    cerr << "Error parsing command line: " << e.what() << std::endl;
    cout << "Usage: mode_manager [MODELFILE]" << std::endl;
    cout << options;
    cout << "Or specify the MODELFILE by ROS parameter 'modelfile'." << std::endl << std::endl;
    return EXIT_FAILURE;
  }

  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  manager = std::make_shared<ModeManager>(modelfile);

  vector<shared_ptr<rclcpp::Subscription<TransitionEvent>>>
  state_sub_;
  vector<shared_ptr<rclcpp::Subscription<ModeEvent>>> mode_sub_;

  vector<shared_ptr<rclcpp::Subscription<TransitionEvent>>>
  state_request_sub_;
  vector<shared_ptr<rclcpp::Subscription<ModeEvent>>>
  mode_request_sub_;

  // Subscribe to lifecycle messages of each node
  for (auto node : manager->inference()->get_nodes()) {
    string lifecycle_topic = "/" + node + "/transition_event";
    string mode_topic = "/" + node + "/mode_event";

    string lifecycle_request_topic = "/" + node + "/transition_request_info";
    string mode_request_topic = "/" + node + "/mode_request_info";

    // Callback for lifecycle transitions
    function<void(TransitionEvent::SharedPtr)> state_callback =
      bind(transition_callback, _1, node);
    auto state_sub = manager->create_subscription<TransitionEvent>(
      lifecycle_topic,
      rclcpp::SystemDefaultsQoS(),
      state_callback);
    state_sub_.push_back(state_sub);

    // Callback for mode transitions
    function<void(ModeEvent::SharedPtr)> mode_callback =
      bind(mode_change_callback, _1, node);
    auto mode_sub = manager->create_subscription<ModeEvent>(
      mode_topic,
      rclcpp::SystemDefaultsQoS(),
      mode_callback);
    mode_sub_.push_back(mode_sub);

    // Callback for lifecycle transitions request info
    state_callback =
      bind(transition_request_callback, _1, node);
    state_sub = manager->create_subscription<TransitionEvent>(
      lifecycle_request_topic,
      rclcpp::SystemDefaultsQoS(),
      state_callback);
    state_request_sub_.push_back(state_sub);

    // Callback for mode transitions request info
    mode_callback =
      bind(mode_request_callback, _1, node);
    mode_sub = manager->create_subscription<ModeEvent>(
      mode_request_topic,
      rclcpp::SystemDefaultsQoS(),
      mode_callback);
    mode_request_sub_.push_back(mode_sub);
  }

  // Subscribe to lifecycle messages of each system
  for (auto system : manager->inference()->get_systems()) {
    string lifecycle_request_topic = "/" + system + "/transition_request_info";
    string mode_request_topic = "/" + system + "/mode_request_info";

    // Callback for lifecycle transitions request info
    function<void(TransitionEvent::SharedPtr)> state_callback =
      bind(transition_request_callback, _1, system);
    auto state_sub = manager->create_subscription<TransitionEvent>(
      lifecycle_request_topic,
      rclcpp::SystemDefaultsQoS(),
      state_callback);
    state_request_sub_.push_back(state_sub);

    // Callback for mode transitions request info
    function<void(ModeEvent::SharedPtr)> mode_callback =
      bind(mode_request_callback, _1, system);
    auto mode_sub = manager->create_subscription<ModeEvent>(
      mode_request_topic,
      rclcpp::SystemDefaultsQoS(),
      mode_callback);
    mode_request_sub_.push_back(mode_sub);
  }

  // Listen for parameter changes
  auto param_sub = manager->create_subscription<ParameterEvent>(
    "/parameter_events",
    rclcpp::ParameterEventsQoS(),
    parameter_event_callback);

  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(manager);
  exe.spin();

  rclcpp::shutdown();

  return 0;
}
