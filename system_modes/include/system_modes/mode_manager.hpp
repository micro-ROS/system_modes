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
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>

#include <lifecycle_msgs/srv/change_state.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <lifecycle_msgs/srv/get_available_states.hpp>
#include <lifecycle_msgs/msg/transition_event.hpp>

#include <map>
#include <string>
#include <memory>
#include <utility>

#include "system_modes/mode_inference.hpp"
#include "system_modes/srv/change_mode.hpp"
#include "system_modes/srv/get_mode.hpp"
#include "system_modes/srv/get_available_modes.hpp"
#include "system_modes/msg/mode_event.hpp"

namespace system_modes
{

class ModeManager : public rclcpp::Node
{
public:
  ModeManager();
  ModeManager(const ModeManager &) = delete;

  std::shared_ptr<ModeInference> inference();

  virtual ~ModeManager() = default;

protected:
  virtual void add_system(const std::string &);
  virtual void add_node(const std::string &);

  // Lifecycle service callbacks
  virtual void on_change_state(
    const std::string &,
    const std::shared_ptr<lifecycle_msgs::srv::ChangeState::Request>,
    std::shared_ptr<lifecycle_msgs::srv::ChangeState::Response>);
  virtual void on_get_state(
    const std::string &,
    std::shared_ptr<lifecycle_msgs::srv::GetState::Response>);
  virtual void on_get_available_states(
    const std::string &,
    std::shared_ptr<lifecycle_msgs::srv::GetAvailableStates::Response>);

  // Mode service callbacks
  virtual void on_change_mode(
    const std::string &,
    const std::shared_ptr<system_modes::srv::ChangeMode::Request>,
    std::shared_ptr<system_modes::srv::ChangeMode::Response>);
  virtual void on_get_mode(
    const std::string &,
    std::shared_ptr<system_modes::srv::GetMode::Response>);
  virtual void on_get_available_modes(
    const std::string &,
    std::shared_ptr<system_modes::srv::GetAvailableModes::Response>);

  virtual bool change_state(
    const std::string &,
    unsigned int,
    bool transitive = true);
  virtual bool change_mode(
    const std::string &,
    const std::string &);

  virtual void change_part_state(
    const std::string &,
    unsigned int);
  virtual void change_part_mode(
    const std::string &,
    const std::string &);

  virtual void publish_transitions();

private:
  std::shared_ptr<ModeInference> mode_inference_;

  // Lifecycle change services
  std::map<std::string, rclcpp::Service<lifecycle_msgs::srv::ChangeState>::SharedPtr>
  state_change_srv_;
  std::map<std::string, rclcpp::Service<lifecycle_msgs::srv::GetState>::SharedPtr>
  get_state_srv_;
  std::map<std::string, rclcpp::Service<lifecycle_msgs::srv::GetAvailableStates>::SharedPtr>
  states_srv_;

  // Mode change services
  std::map<std::string, rclcpp::Service<system_modes::srv::ChangeMode>::SharedPtr>
  mode_change_srv_;
  std::map<std::string, rclcpp::Service<system_modes::srv::GetMode>::SharedPtr>
  get_mode_srv_;
  std::map<std::string, rclcpp::Service<system_modes::srv::GetAvailableModes>::SharedPtr>
  modes_srv_;

  // Lifecycle / Mode / Parameter service clients
  std::map<std::string, rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr>
  state_change_clients_;
  std::map<std::string, rclcpp::Client<system_modes::srv::ChangeMode>::SharedPtr>
  mode_change_clients_;
  std::map<std::string, rclcpp::AsyncParametersClient::SharedPtr>
  param_change_clients_;

  // Lifecycle transition publisher
  std::map<std::string, rclcpp::Publisher<lifecycle_msgs::msg::TransitionEvent>::SharedPtr>
  transition_pub_;
  std::map<std::string, rclcpp::Publisher<lifecycle_msgs::msg::TransitionEvent>::SharedPtr>
  state_request_pub_;

  // Mode transition publisher
  std::map<std::string, rclcpp::Publisher<system_modes::msg::ModeEvent>::SharedPtr>
  mode_transition_pub_;
  std::map<std::string, rclcpp::Publisher<system_modes::msg::ModeEvent>::SharedPtr>
  mode_request_pub_;

  // Remember states and modes of the systems
  std::map<std::string, StateAndMode> current_modes_;

  // Timer to check for and publish recent transitions
  rclcpp::TimerBase::SharedPtr transition_timer_;
};

}  // namespace system_modes
