# ROS 2 System Modes Messages

This package provides the message types and services for the System Modes Library.

## Message Types

* [msg/Mode.msg](./msg/Mode.msg) - Mode definition, along the lines of lifecycle_msgs [State](https://github.com/ros2/rcl_interfaces/blob/master/lifecycle_msgs/msg/State.msg), identifying a mode by a string.
* [msg/ModeEvent.msg](./msg/Mode.msg) - Notifies about the transition of a system mode, along the lines of lifecycle_msgs [TransitionEvent](https://github.com/ros2/rcl_interfaces/blob/master/lifecycle_msgs/msg/TransitionEvent.msg).

## Services

1. [srv/GetMode.srv](./srv/GetMode.srv) - Requests the current mode of a (sub-)system or node, along the lines of lifecycle_msgs [GetState](https://github.com/ros2/rcl_interfaces/blob/master/lifecycle_msgs/srv/GetState.srv)
2. [srv/GetAvailableModes.srv](./srv/GetAvailableModes.srv) - Requests all available modes of a (sub-)system or node, along the lines of lifecycle_msgs [GetAvailableStates](https://github.com/ros2/rcl_interfaces/blob/master/lifecycle_msgs/srv/GetAvailableStates.srv)
3. [srv/ChangeMode.srv](./srv/ChangeMode.srv) - Requests a change to a certain mode, along the lines of lifecycle_msgs [ChangeState](https://github.com/ros2/rcl_interfaces/blob/master/lifecycle_msgs/srv/ChangeState.srv)
