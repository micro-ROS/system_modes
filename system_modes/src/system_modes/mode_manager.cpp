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
#include "system_modes/mode_manager.hpp"

#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <lifecycle_msgs/msg/transition_event.hpp>

#include <map>
#include <string>
#include <chrono>
#include <vector>
#include <memory>
#include <iomanip>
#include <utility>
#include <iostream>
#include <algorithm>
#include <functional>

#include "system_modes/msg/mode_event.hpp"

using std::string;
using std::out_of_range;
using std::chrono::seconds;

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

using lifecycle_msgs::msg::State;
using lifecycle_msgs::msg::Transition;
using lifecycle_msgs::msg::TransitionEvent;
using lifecycle_msgs::srv::GetState;
using lifecycle_msgs::srv::ChangeState;
using lifecycle_msgs::srv::GetAvailableStates;

using system_modes::msg::ModeEvent;
using system_modes::srv::ChangeMode;
using system_modes::srv::GetAvailableModes;

namespace system_modes
{

ModeManager::ModeManager(const string & model_path)
: Node("__mode_manager"),
  mode_inference_(nullptr),
  state_change_srv_(), get_state_srv_(), states_srv_(),
  mode_change_srv_(), get_mode_srv_(), modes_srv_(),
  state_change_clients_(), mode_change_clients_(),
  state_request_pub_(), mode_request_pub_()
{
  declare_parameter("modelfile", rclcpp::ParameterValue(std::string("")));
  if (model_path.empty()) {
    rclcpp::Parameter parameter = get_parameter("modelfile");
    std::string alt_model_path = parameter.get_value<rclcpp::ParameterType::PARAMETER_STRING>();
    if (alt_model_path.empty()) {
      throw std::invalid_argument("Need path to model file.");
    }
    mode_inference_ = std::make_shared<ModeInference>(alt_model_path);
  } else {
    mode_inference_ = std::make_shared<ModeInference>(model_path);
  }

  for (auto system : this->mode_inference_->get_systems()) {
    this->add_system(system);

    RCLCPP_INFO(get_logger(), "- system '%s'", system.c_str());
    RCLCPP_INFO(get_logger(), "  - %s/change_state", system.c_str());
    RCLCPP_INFO(get_logger(), "  - %s/get_state", system.c_str());
    RCLCPP_INFO(get_logger(), "  - %s/get_available_states", system.c_str());
    RCLCPP_INFO(get_logger(), "  - %s/change_mode", system.c_str());
    RCLCPP_INFO(get_logger(), "  - %s/get_mode", system.c_str());
    RCLCPP_INFO(get_logger(), "  - %s/get_available_modes", system.c_str());
  }
  for (auto node : this->mode_inference_->get_nodes()) {
    this->add_node(node);

    RCLCPP_INFO(get_logger(), "- node '%s'", node.c_str());
    RCLCPP_INFO(get_logger(), "  - %s/change_mode", node.c_str());
    RCLCPP_INFO(get_logger(), "  - %s/get_mode", node.c_str());
    RCLCPP_INFO(get_logger(), "  - %s/get_available_modes", node.c_str());
  }
}

std::shared_ptr<ModeInference>
ModeManager::inference()
{
  return this->mode_inference_;
}

void
ModeManager::add_system(const std::string & system)
{
  string service_name;

  // Lifecycle services
  service_name = system + "/change_state";
  std::function<void(const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<ChangeState::Request>,
    std::shared_ptr<ChangeState::Response>)> state_change_callback =
    std::bind(&ModeManager::on_change_state, this, system, _2, _3);
  this->state_change_srv_[system] = this->create_service<ChangeState>(
    service_name,
    state_change_callback);

  service_name = system + "/get_state";
  std::function<void(const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<GetState::Request>,
    std::shared_ptr<GetState::Response>)> state_get_callback =
    std::bind(&ModeManager::on_get_state, this, system, _3);
  this->get_state_srv_[system] = this->create_service<GetState>(
    service_name,
    state_get_callback);

  service_name = system + "/get_available_states";
  std::function<void(const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<GetAvailableStates::Request>,
    std::shared_ptr<GetAvailableStates::Response>)> state_avail_callback =
    std::bind(&ModeManager::on_get_available_states, this, system, _3);
  this->states_srv_[system] = this->create_service<GetAvailableStates>(
    service_name,
    state_avail_callback);

  // Mode services
  service_name = system + "/change_mode";
  this->mode_change_srv_[system] = this->create_service<ChangeMode>(
    service_name, std::bind(&ModeManager::on_change_mode, this, _1, _2, _3));

  service_name = system + "/get_mode";
  this->get_mode_srv_[system] =
    this->create_service<system_modes::srv::GetMode>(
    service_name,
    std::bind(&ModeManager::on_get_mode, this, _1, _2, _3));

  service_name = system + "/get_available_modes";
  this->modes_srv_[system] = this->create_service<GetAvailableModes>(
    service_name, std::bind(&ModeManager::on_get_available_modes, this, _1, _2, _3));

  // Lifecycle change clients
  service_name = system + "/change_state";
  this->state_change_clients_[system] = this->create_client<ChangeState>(service_name);

  // Mode change clients
  service_name = system + "/change_mode";
  this->mode_change_clients_[system] = this->create_client<ChangeMode>(service_name);

  // State request publisher
  service_name = system + "/transition_request_info";
  this->state_request_pub_[system] = this->create_publisher<TransitionEvent>(service_name, 1);

  // Mode request publisher
  service_name = system + "/mode_request_info";
  this->mode_request_pub_[system] = this->create_publisher<ModeEvent>(service_name, 1);
}

void
ModeManager::add_node(const std::string & node)
{
  string service_name;

  // Mode services
  service_name = node + "/change_mode";
  this->mode_change_srv_[node] = this->create_service<ChangeMode>(
    service_name, std::bind(&ModeManager::on_change_mode, this, _1, _2, _3));

  service_name = node + "/get_mode";
  this->get_mode_srv_[node] =
    this->create_service<system_modes::srv::GetMode>(
    service_name,
    std::bind(&ModeManager::on_get_mode, this, _1, _2, _3));

  service_name = node + "/get_available_modes";
  this->modes_srv_[node] = this->create_service<GetAvailableModes>(
    service_name, std::bind(&ModeManager::on_get_available_modes, this, _1, _2, _3));

  // Lifecycle change clients
  service_name = node + "/change_state";
  this->state_change_clients_[node] = this->create_client<ChangeState>(service_name);

  // Parameter change clients
  service_name = node + "/set_parameters_atomically";
  this->param_change_clients_[node] = std::make_shared<rclcpp::AsyncParametersClient>(
    this->get_node_base_interface(),
    this->get_node_topics_interface(),
    this->get_node_graph_interface(),
    this->get_node_services_interface(),
    node);

  // State request publisher
  service_name = node + "/transition_request_info";
  this->state_request_pub_[node] = this->create_publisher<TransitionEvent>(service_name, 1);

  // Mode request publisher
  service_name = node + "/mode_request_info";
  this->mode_request_pub_[node] = this->create_publisher<ModeEvent>(service_name, 1);
}

void
ModeManager::on_change_state(
  const std::string & system,
  const std::shared_ptr<ChangeState::Request> request,
  std::shared_ptr<ChangeState::Response> response)
{
  RCLCPP_INFO(
    this->get_logger(),
    "-> request for transition of %s: %s",
    system.c_str(),
    request->transition.label.c_str());

  // We can't wait for the state/mode transitions
  response->success = this->change_state(system, request->transition.id);
}

void
ModeManager::on_get_state(
  const std::string & system,
  std::shared_ptr<GetState::Response> response)
{
  // TODO(anordman): to be on the safe side, don't use the node name from
  //       the request, but bind it to the callback instead
  RCLCPP_INFO(this->get_logger(), "-> callback get_state of %s", system.c_str());
  try {
    auto stateAndMode = this->mode_inference_->infer(system);
    if (stateAndMode.state == 0) {
      response->current_state.id = State::PRIMARY_STATE_UNCONFIGURED;
      response->current_state.label = "unconfigured";
    } else {
      response->current_state.id = stateAndMode.state;
      response->current_state.label = state_label_(stateAndMode.state);
    }
  } catch (std::exception & ex) {
    response->current_state.id = State::PRIMARY_STATE_UNKNOWN;
    response->current_state.label = "unknown";
  }
  RCLCPP_INFO(
    this->get_logger(),
    " state %s(%d)",
    response->current_state.label.c_str(),
    response->current_state.id);
}

void
ModeManager::on_get_available_states(
  const std::string & system,
  std::shared_ptr<GetAvailableStates::Response> response)
{
  // TODO(anordman): to be on the safe side, don't use the node name from
  //       the request, but bind it to the callback instead
  RCLCPP_INFO(
    this->get_logger(),
    "-> callback get_available_states of %s",
    system.c_str());

  // TODO(anordman): Don't hard-code: Which are the available states for systems, though?
  State inactive;
  inactive.id = State::PRIMARY_STATE_INACTIVE;
  inactive.label = "inactive";
  response->available_states.push_back(inactive);
  State active;
  active.id = State::PRIMARY_STATE_ACTIVE;
  active.label = "active";
  response->available_states.push_back(active);
}

void
ModeManager::on_change_mode(
  const std::shared_ptr<rmw_request_id_t>,
  const std::shared_ptr<ChangeMode::Request> request,
  std::shared_ptr<ChangeMode::Response> response)
{
  RCLCPP_INFO(
    this->get_logger(),
    "-> callback change_mode of %s to %s",
    request->node_name.c_str(),
    request->mode_name.c_str());

  // We can't wait for the state/mode transitions
  response->success = this->change_mode(request->node_name, request->mode_name);
}

void
ModeManager::on_get_mode(
  const std::shared_ptr<rmw_request_id_t>,
  const std::shared_ptr<system_modes::srv::GetMode::Request> request,
  std::shared_ptr<system_modes::srv::GetMode::Response> response)
{
  // TODO(anordman): to be on the safe side, don't use the node name from
  //       the request, but bind it to the callback instead
  RCLCPP_INFO(this->get_logger(), "-> callback get_mode of %s", request->node_name.c_str());
  try {
    auto stateAndMode = this->mode_inference_->infer(request->node_name);
    response->current_mode = stateAndMode.mode;
  } catch (std::exception & ex) {
    response->current_mode = "unknown";
  }
  RCLCPP_INFO(this->get_logger(), " mode %s", response->current_mode.c_str());
}

void
ModeManager::on_get_available_modes(
  const std::shared_ptr<rmw_request_id_t>,
  const std::shared_ptr<GetAvailableModes::Request> request,
  std::shared_ptr<GetAvailableModes::Response> response)
{
  // TODO(anordman): to be on the safe side, don't use the node name from
  //       the request, but bind it to the callback instead
  RCLCPP_INFO(
    this->get_logger(),
    "-> callback get_available_modes of %s",
    request->node_name.c_str());
  try {
    response->available_modes = mode_inference_->get_available_modes(request->node_name);
  } catch (std::exception & ex) {
    RCLCPP_INFO(this->get_logger(), " unknown");
  }
}


bool
ModeManager::change_state(
  const std::string & node_name,
  unsigned int transition_id,
  bool transitive)
{
  RCLCPP_INFO(
    this->get_logger(),
    " changing state of %s: %s",
    node_name.c_str(),
    transition_label_(transition_id).c_str());

  // Publish info about this request
  auto info = std::make_shared<TransitionEvent>();
  if (this->current_modes_.find(node_name) != this->current_modes_.end()) {
    info->start_state.label = this->current_modes_.at(node_name).first;
  }
  info->transition.label = transition_label_(transition_id);
  info->transition.id = transition_id;
  info->goal_state.id = goal_state_(transition_id);
  info->goal_state.label = state_label_(info->goal_state.id);
  RCLCPP_DEBUG(
    this->get_logger(),
    " published info about request to %s %s",
    info->transition.label.c_str(),
    node_name.c_str());
  this->state_request_pub_[node_name]->publish(*info);

  if (!transitive) {
    return true;
  }

  // if state is 'active', go to default mode instead
  if (goal_state_(transition_id) != State::PRIMARY_STATE_ACTIVE) {
    RCLCPP_DEBUG(this->get_logger(), "  not active, so simple state transition");
    // modify node_name in the request and forward to the parts
    for (auto part : this->mode_inference_->get_all_parts_of(node_name)) {
      this->change_part_state(part, transition_id);
    }
  } else {
    RCLCPP_DEBUG(this->get_logger(), "  active, so trigger mode transition to __DEFAULT__");
    this->change_mode(node_name, DEFAULT_MODE);
  }
  return true;
}

bool
ModeManager::change_mode(
  const std::string & node_name,
  const std::string & mode_name)
{
  RCLCPP_INFO(
    this->get_logger(),
    " changing mode of %s to %s",
    node_name.c_str(),
    mode_name.c_str());

  auto new_mode = this->mode_inference_->get_mode(node_name, mode_name);
  if (!new_mode) {
    RCLCPP_ERROR(
      this->get_logger(),
      " mode change failed, unknown mode %s of %s",
      mode_name.c_str(),
      node_name.c_str());
    return false;
  }

  // Publish info about this request
  auto info = std::make_shared<ModeEvent>();
  info->goal_mode.label = mode_name;
  this->mode_request_pub_[node_name]->publish(*info);
  RCLCPP_DEBUG(
    this->get_logger(),
    " published info about request to change %s to %s",
    node_name.c_str(),
    info->goal_mode.label.c_str());

  this->current_modes_.emplace(
    node_name,
    std::make_pair(State::PRIMARY_STATE_ACTIVE, mode_name.c_str()));

  auto nodes = this->mode_inference_->get_nodes();
  if (std::find(nodes.begin(), nodes.end(), node_name) != nodes.end()) {
    // if node, change parameters accordingly
    this->change_part_mode(node_name, mode_name);
    return true;
  }

  auto systems = this->mode_inference_->get_systems();
  if (std::find(systems.begin(), systems.end(), node_name) != systems.end()) {
    // if system, change parts accordingly
    for (auto part : new_mode->get_parts()) {
      auto stateAndMode = new_mode->get_part_mode(part);

      if (stateAndMode.state != State::PRIMARY_STATE_ACTIVE) {
        this->change_part_state(part, Transition::TRANSITION_DEACTIVATE);
      } else {
        // TODO(anordman): This is not always correct. Find the correct
        // state/transition and mode via mode inference
        this->change_part_state(part, Transition::TRANSITION_ACTIVATE);
        if (stateAndMode.mode.empty()) {
          this->change_part_mode(part, DEFAULT_MODE);
        } else {
          this->change_part_mode(part, stateAndMode.mode);
        }
      }
    }

    return true;
  }

  return false;
}

void
ModeManager::change_part_state(const string & node, unsigned int transition)
{
  RCLCPP_INFO(
    this->get_logger(),
    " changing state of %s: %s",
    node.c_str(),
    transition_label_(transition).c_str());

  // Request
  auto request = std::make_shared<ChangeState::Request>();
  request->transition.id = transition;
  request->transition.label = transition_label_(transition);

  // Publish info about this request
  auto info = std::make_shared<TransitionEvent>();
  info->transition.label = transition_label_(transition);
  info->transition.id = transition;
  if (this->current_modes_.find(node) != this->current_modes_.end()) {
    info->start_state.label = this->current_modes_.at(node).first;
    info->start_state.id = state_id_(info->start_state.label);
  }
  info->goal_state.id = goal_state_(info->transition.id);
  info->goal_state.label = state_label_(info->goal_state.id);
  RCLCPP_DEBUG(
    this->get_logger(),
    " published info about request to %s %s",
    transition_label_(transition).c_str(),
    node.c_str());
  this->state_request_pub_[node]->publish(*info);

  // Don't wait for the result, we can't do this inside a service
  this->state_change_clients_[node]->async_send_request(request);
}

void
ModeManager::change_part_mode(const string & node, const string & mode)
{
  RCLCPP_INFO(
    this->get_logger(),
    " changing mode of %s to %s",
    node.c_str(),
    mode.c_str());

  auto request = std::make_shared<ChangeMode::Request>();
  request->mode_name = mode;

  // Publish info about this request
  auto info = std::make_shared<ModeEvent>();
  info->goal_mode.label = mode;
  this->mode_request_pub_[node]->publish(*info);
  RCLCPP_DEBUG(
    this->get_logger(),
    " published info about request to change %s to %s",
    node.c_str(),
    info->goal_mode.label.c_str());

  auto known_systems = mode_inference_->get_systems();
  auto known_nodes = mode_inference_->get_nodes();
  if (std::find(known_systems.begin(), known_systems.end(), node) != known_systems.end()) {
    // It's a system, call the mode change
    // Don't wait for the result, we can't do this inside a service
    RCLCPP_DEBUG(
      this->get_logger(),
      "  sending change mode request to change %s to %s",
      node.c_str(),
      info->goal_mode.label.c_str());
    this->mode_change_clients_[node]->async_send_request(request);
  } else if (std::find(known_nodes.begin(), known_nodes.end(), node) != known_nodes.end()) {
    // It's a node, change parameters accordingly
    auto new_mode = this->mode_inference_->get_mode(node, mode);
    for (auto p : new_mode->get_parameter_names()) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "  sending parameter change request to %s: %s",
        node.c_str(),
        p.c_str());
    }
    this->param_change_clients_[node]->set_parameters(new_mode->get_parameters());
  }
}

}  // namespace system_modes
