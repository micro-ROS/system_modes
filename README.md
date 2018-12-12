# ROS 2 System Modes

This repository explores a system modes concept that is implemented for ROS 2 in two packages:
* [system_modes](./system_modes/) provides a library for system mode inference, a mode manager, and a mode monitor
* [system_modes_examples](./system_modes_examples/) implements a simple example

For further information, please contact [Arne Nordmann](https://github.com/norro) or [Ralph Lange](https://github.com/ralph-lange).

## Purpose of the Project

This software is not ready for production use. It has neither been developed nor
tested for a specific use case. However, the license conditions of the
applicable Open Source licenses allow you to adapt the software to your needs.
Before using it in a safety relevant setting, make sure that the software
fulfills your requirements and adjust it according to any applicable safety
standards, e.g., ISO 26262.

## How to Build, Test, Install, and Use

To try system modes in your ROS 2 system, two pull request / patches need to be applied to your local ROS 2 installation to add information on the source node to parameter events:
* rcl_interfaces [pull request 51](https://github.com/ros2/rcl_interfaces/pull/51) ([patch](https://github.com/ros2/rcl_interfaces/pull/51.patch))
* rclcpp [pull request 584](https://github.com/ros2/rclcpp/pull/584) ([patch](https://github.com/ros2/rclcpp/pull/584.patch))

After you clone this repository into your ROS 2 src folder and applied these two patches, you may build and install the [system_modes](./system_modes/) package and the [system_modes_examples](./system_modes_examples/) package:  
$ `colcon build --packages-select-regex system_modes`

Have a look at the [system_modes_examples](./system_modes_examples/) documentation to try your installation.

## License

ROS 2 System Modes are open-sourced under the Apache-2.0 license. See the
[LICENSE](LICENSE) file for details.

For a list of other open-source components included in ROS 2 system_modes,
see the file [3rd-party-licenses.txt](3rd-party-licenses.txt).

## Known Issues/Limitations

Please notice the following issues/limitations:

* Currently, this package depends on [rcl_interfaces pull request 51](https://github.com/ros2/rcl_interfaces/pull/51) and [rclcpp pull request 584](https://github.com/ros2/rclcpp/pull/584).
* Currently, (sub-)systems managed by the mode manager are not recognized by the `ros2 lifecycle` tool (*"Node not found"*). So to trigger lifecycle transitions in (sub-)systems, you have to go with the `ros2 service call` tool. Check the [system_modes_examples](./system_modes_examples/) documentation for example calls.

## Acknowledgments

This activity has received funding from the European Research Council (ERC) under the European Union's Horizon 2020 research and innovation programme (grant agreement n° 780785).
