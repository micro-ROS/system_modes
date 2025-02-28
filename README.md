# ROS 2 System Modes

[![License](https://img.shields.io/badge/License-Apache%202-blue.svg)](https://github.com/micro-ROS/system_modes/blob/master/LICENSE)
[![Build status](http://build.ros2.org/job/Hdev__system_modes__ubuntu_jammy_amd64/badge/icon?subject=Build%20farm%3A%20Humble)](http://build.ros2.org/job/Hdev__system_modes__ubuntu_jammy_amd64/)
[![Build status](http://build.ros2.org/job/Jdev__system_modes__ubuntu_noble_amd64/badge/icon?subject=Build%20farm%3A%20Jazzy)](http://build.ros2.org/job/Jdev__system_modes__ubuntu_noble_amd64/)
[![Build status](http://build.ros2.org/job/Rdev__system_modes__ubuntu_noble_amd64/badge/icon?subject=Build%20farm%3A%20Rolling)](http://build.ros2.org/job/Rdev__system_modes__ubuntu_noble_amd64/)
[![Build status](https://github.com/micro-ROS/system_modes/workflows/CI%3A%20humble%2C%20jazzy%2C%20rolling/badge.svg)](https://github.com/micro-ROS/system_modes/actions)
[![Code coverage](https://codecov.io/gh/micro-ROS/system_modes/branch/master/graph/badge.svg)](https://codecov.io/gh/micro-ROS/system_modes)

This repository explores a system modes concept that is implemented for ROS 2 in these packages:
* [system_modes_msgs](./system_modes_msgs/) provides the message types and services for system modes
* [system_modes](./system_modes/) provides a library for system mode inference, a mode manager, and a mode monitor
* [system_modes_examples](./system_modes_examples/) implements a simple example
* [launch_system_modes](./launch_system_modes/) launch actions, events, and event handlers for system modes
* [test_launch_system_modes](./test_launch_system_modes/) launch test for the launch_system_modes` package

For further information, please contact [Arne Nordmann](https://github.com/norro) or [Ralph Lange](https://github.com/ralph-lange).

## Purpose of the Project

This software is not ready for production use. It has neither been developed nor
tested for a specific use case. However, the license conditions of the
applicable Open Source licenses allow you to adapt the software to your needs.
Before using it in a safety relevant setting, make sure that the software
fulfills your requirements and adjust it according to any applicable safety
standards, e.g., ISO 26262.

## How to Build, Test, Install, and Use

After you cloned this repository into your ROS 2 workspace folder, you may build and install the [system_modes](./system_modes/) package and the [system_modes_examples](./system_modes_examples/) package using colcon:
$ `colcon build --packages-select-regex system_modes`

Have a look at the [system_modes_examples](./system_modes_examples/) documentation to try your installation.

For using this package and designing system modes for your system, please refer to the [How to Apply](./system_modes/README.md#how-to-apply) section.

## License

ROS 2 System Modes are open-sourced under the Apache-2.0 license. See the
[LICENSE](LICENSE) file for details.

For a list of other open-source components included in ROS 2 system_modes,
see the file [3rd-party-licenses.txt](3rd-party-licenses.txt).

## Quality assurance

The colcon_test tool is used for quality assurances, which includes cpplint, uncrustify, flake8, xmllint and various other tools.

Unit tests based on [gtest](https://github.com/google/googletest) are located in the [./system_modes/test](system_modes/test) folder.

## Known Issues/Limitations

Please notice the following issues/limitations:

* Currently, (sub-)systems managed by the mode manager are not recognized by the `ros2 lifecycle` tool (*"Node not found"*). So to trigger lifecycle transitions in (sub-)systems, you have to go with the `ros2 service call` tool. Check the [system_modes_examples](./system_modes_examples/) documentation for example calls.
* The [Error Handling and Rules](./system_modes/README.md#error-handling-and-rules-experimental) feature is still experimental and might be subject to major changes. However, if no rules are specified in the model file, this feature is not used.
* The mode inference and the error handling and rules feature do not work as intended if some of the involved nodes are non-lifecycle nodes.

## Acknowledgments

This activity has received funding from the European Research Council (ERC) under the European Union's Horizon 2020 research and innovation programme (grant agreement n° 780785).
