General information about this repository, including legal information, build instructions and known issues/limitations, can be found in the [README](../README.md) of the repository root.

# The launch_system_modes package

This [ROS 2](https://index.ros.org/doc/ros2/) package provides a launch actions, events, and event handlers for the use of the [system_modes](../system_modes/) package.

General information about this repository, including legal information, project context, build instructions and known issues/limitations, are given in [README.md](../README.md) in the repository root.

## Launch System Modes Package

The actions, events, and event handlers implemented for system modes are:

### Actions

Two launch actions are implemented for system modes:

* `System`: Declares a *system*, consisting of further system parts, allowing system mode specific launch events and event handlers.
* `Node`: Declares a *node*, i.e. a lifecycle node with system modes. It inherits from the [launch_ros/lifecycle_node](https://github.com/ros2/launch_ros/blob/master/launch_ros/launch_ros/actions/lifecycle_node.py) action and allows further system mode specific events and event handlers.

### Events

* `ChangeMode`: Trigger a mode change in a `System` or `Node`
* `ChangeState`: Trigger a state transition in a `System`, since [launch_ros/ChangeState](https://github.com/ros2/launch_ros/blob/master/launch_ros/launch_ros/events/lifecycle/change_state.py) only works for lifecycle nodes, not systems.
* `ModeChanged`: Emitted when a `System` or `Node` changed its mode.
* `StateTransition`: Emitted when a `System` changed its state, since [launch_ros/StateTransition](https://github.com/ros2/launch_ros/blob/master/launch_ros/launch_ros/events/lifecycle/state_transition.py) only works for lifecycle nodes, not systems.

### Event Handlers

* `OnModeChanged`: Event handler for mode changes of a `System` or `Node`
* `OnStateTransition`: Event handler for state transitions of a `System`

## Examples

Two examples show the use of launch_system_modes:

1. [system_modes_examples/launch/example_system_start_drive_base.launch.py](../system_modes_examples/launch/example_system_start_drive_base.launch.py) starts an *actuation* system with two system parts, the nodes *drive_base* and *manipulator*. It will then:
    1. trigger a *configure* transition for the *drive_base* system part ([lines 62 - 65](../system_modes_examples/launch/example_system_start_drive_base.launch.py#L62-L65))
    1. a state change handler ([lines 86 - 90](../system_modes_examples/launch/example_system_start_drive_base.launch.py#L86-L90)) notices the successful transition and triggers an *activate* transition for the *drive_base* system part ([lines 67 - 71](../system_modes_examples/launch/example_system_start_drive_base.launch.py#L67-L71))
    1. another state change handler ([lines 92 - 96](../system_modes_examples/launch/example_system_start_drive_base.launch.py#L92-L96)) notices the successful transition to *active* and triggers a mode change of the *drive_base* system part to its default mode ([lines 73 - 77](../system_modes_examples/launch/example_system_start_drive_base.launch.py#L73-L77))
    1. a mode change handler ([lines 98 - 102](../system_modes_examples/launch/example_system_start_drive_base.launch.py#L98-L102)) notices the successful transition to the default mode and triggers a mode change of the *drive_base* system part to its *FAST* mode ([lines 79 - 83](../system_modes_examples/launch/example_system_start_drive_base.launch.py#L79-L83))
1. [system_modes_examples/launch/example_system_started.launch.py](../system_modes_examples/launch/example_system_started.launch.py) starts the same system, but uses according events and event handlers for the *system* instead. It will:
    1. trigger a *configure* transition for the *actuation* system ([lines 60 - 63](../system_modes_examples/launch/example_system_started.launch.py#L62-L65))
    1. a state change handler ([lines 78 - 82](../system_modes_examples/launch/example_system_started.launch.py#L78-L82)) notices the successful transition and triggers an *activate* transition for the *actuation* system ([lines 65 - 69](../system_modes_examples/launch/example_system_started.launch.py#L65-L69))
    1. a mode change handler ([lines 84 - 88](../system_modes_examples/launch/example_system_started.launch.py#L84-L88)) notices the successful transition to the default mode and triggers a mode change of the *actuation* system to its *PERMORMANCE* mode ([lines 71 - 75](../system_modes_examples/launch/example_system_started.launch.py#L71-L75))
