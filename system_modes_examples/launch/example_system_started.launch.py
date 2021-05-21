# Copyright (c) 2020 - for information on the respective copyright owner
# see the NOTICE file and/or the repository https://github.com/micro-ROS/system_modes.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import ament_index_python.packages

import launch
import launch.actions
import launch.events
import launch.launch_description_sources
import launch.substitutions

import launch_system_modes.actions
import launch_system_modes.event_handlers
import launch_system_modes.events

import lifecycle_msgs


def generate_launch_description():
    modelfile = (ament_index_python.packages.get_package_share_directory('system_modes_examples') +
                 '/example_modes.yaml')

    # Setup
    mode_manager = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            ament_index_python.packages.get_package_share_directory(
                'system_modes') + '/launch/mode_manager.launch.py'),
        launch_arguments={'modelfile': modelfile}.items())

    actuation = launch_system_modes.actions.System(
            name='actuation',
            namespace='')

    drive_base = launch_system_modes.actions.Node(
            package='system_modes_examples',
            executable='drive_base',
            name='drive_base',
            namespace='',
            output='screen')

    manipulator = launch_system_modes.actions.Node(
            package='system_modes_examples',
            executable='manipulator',
            name='manipulator',
            namespace='',
            output='screen')

    # Startup
    actuation_configure = launch.actions.EmitEvent(
        event=launch_system_modes.events.ChangeState(
            system_part_matcher=launch.events.matchers.matches_action(actuation),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE))

    actuation_activate = launch.actions.EmitEvent(
        event=launch_system_modes.events.ChangeState(
            system_part_matcher=launch.events.matchers.matches_action(actuation),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
        ))

    actuation_change_mode_to_PERFORMANCE = launch.actions.EmitEvent(
        event=launch_system_modes.events.ChangeMode(
            system_part_matcher=launch.events.matchers.matches_action(actuation),
            mode_name='PERFORMANCE',
        ))

    # Handlers
    on_inactive_handler = launch.actions.RegisterEventHandler(
        launch_system_modes.event_handlers.OnStateTransition(
            target_system_part=actuation,
            goal_state='inactive',
            entities=[actuation_activate]))

    on_DEFAULT_mode = launch.actions.RegisterEventHandler(
        launch_system_modes.event_handlers.OnModeChanged(
            target_system_part=actuation,
            goal_mode='__DEFAULT__',
            entities=[actuation_change_mode_to_PERFORMANCE]))

    description = launch.LaunchDescription()
    description.add_action(on_inactive_handler)
    description.add_action(on_DEFAULT_mode)
    description.add_action(mode_manager)
    description.add_action(actuation)
    description.add_action(drive_base)
    description.add_action(manipulator)
    description.add_action(actuation_configure)
    return description
