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
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>

#include <memory>
#include <vector>

using lifecycle_msgs::msg::State;
using lifecycle_msgs::msg::Transition;
using rclcpp_lifecycle::LifecycleNode;

using namespace std::chrono_literals;

namespace system_modes
{
namespace examples
{

// Manipulator node with two non-default modes: weak and strong
class Manipulator : public LifecycleNode
{
public:
  Manipulator()
  : LifecycleNode("manipulator")
  {
    RCLCPP_INFO(get_logger(), "Constructed lifecycle node '%s'", this->get_name());

    // Parameter declaration
    this->declare_parameter("max_torque", 0.0);

    auto param_change_callback =
      [this](std::vector<rclcpp::Parameter> parameters) -> rcl_interfaces::msg::SetParametersResult
      {
        auto result = rcl_interfaces::msg::SetParametersResult();
        result.successful = true;
        for (auto parameter : parameters) {
          RCLCPP_INFO(
            this->get_logger(),
            "parameter '%s' is now: %s",
            parameter.get_name().c_str(),
            parameter.value_to_string().c_str());
        }
        return result;
      };

    param_change_callback_handle_ = this->add_on_set_parameters_callback(param_change_callback);
  }

  Manipulator(const Manipulator &) = delete;
  ~Manipulator() = default;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "%s on_configure()", this->get_name());

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "%s on_activate()", this->get_name());

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "%s on_deactivate()", this->get_name());

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "%s on_cleanup()", this->get_name());

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

private:
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr param_change_callback_handle_;
};

}  // namespace examples
}  // namespace system_modes

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exe;

  auto manipulator = std::make_shared<system_modes::examples::Manipulator>();
  exe.add_node(manipulator->get_node_base_interface());
  exe.spin();

  rclcpp::shutdown();

  return 0;
}
