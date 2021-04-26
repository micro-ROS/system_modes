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

import sys
import ament_index_python.packages

import launch
import launch.actions
import launch.events

import launch.launch_description_sources
import launch.substitutions

import launch_ros.actions
import launch_ros.events
import launch_ros.events.lifecycle

import launch_system_modes.actions
import launch_system_modes.events
import launch_system_modes.event_handlers

import lifecycle_msgs

def generate_launch_description():
    modelfile = (ament_index_python.packages.get_package_share_directory('system_modes_examples') +
                 '/example_modes.yaml')

    try:
        # Setup
        mode_manager = launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                ament_index_python.packages.get_package_share_directory(
                    'system_modes') + '/launch/mode_manager.launch.py'),
            launch_arguments={'modelfile': modelfile}.items())

        system = launch_system_modes.actions.System(
                name='sys',
                namespace='')

        drive_base = launch_system_modes.actions.SystemPart(
                package='system_modes_examples',
                executable='drive_base',
                name='drive_base',
                namespace='',
                parameters=[{'modelfile': modelfile}],
                output='screen')

        manipulator = launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                ament_index_python.packages.get_package_share_directory(
                    'system_modes_examples') + '/launch/manipulator.launch.py'),
            launch_arguments={'modelfile': modelfile}.items())

        # Startup
        system_configure = launch.actions.EmitEvent(
            event=launch_ros.events.lifecycle.ChangeState(
                lifecycle_node_matcher=launch.events.matchers.matches_action(system),
                transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE))

        drive_base_configure = launch.actions.EmitEvent(
            event=launch_ros.events.lifecycle.ChangeState(
                lifecycle_node_matcher=launch.events.matchers.matches_action(drive_base),
                transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE))

        drive_base_activate = launch.actions.EmitEvent(
            event=launch_ros.events.lifecycle.ChangeState(
                lifecycle_node_matcher=launch.events.matchers.matches_action(drive_base),
                transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
            ))

        system_change_mode_to_DEFAULT = launch.actions.EmitEvent(
            event=launch_system_modes.events.ChangeMode(
                lifecycle_node_matcher=launch.events.matchers.matches_action(system),
                mode_name='__DEFAULT__',
            ))
        drive_base_change_mode_to_DEFAULT = launch.actions.EmitEvent(
            event=launch_system_modes.events.ChangeMode(
                lifecycle_node_matcher=launch.events.matchers.matches_action(drive_base),
                mode_name='__DEFAULT__',
            ))
        drive_base_change_mode_to_FAST = launch.actions.EmitEvent(
            event=launch_system_modes.events.ChangeMode(
                lifecycle_node_matcher=launch.events.matchers.matches_action(drive_base),
                mode_name='FAST',
            ))

        # Handlers
        on_inactive_handler = launch.actions.RegisterEventHandler(
            launch_ros.event_handlers.OnStateTransition(
                target_lifecycle_node=drive_base,
                goal_state='inactive',
                entities=[drive_base_activate]))
        on_active_handler = launch.actions.RegisterEventHandler(
            launch_ros.event_handlers.OnStateTransition(
                target_lifecycle_node=drive_base,
                goal_state='active',
                entities=[system_change_mode_to_DEFAULT]))
        on_FAST_mode = launch.actions.RegisterEventHandler(
            launch_system_modes.event_handlers.OnModeChanged(
                target_system_part=drive_base,
                goal_mode='__DEFAULT__',
                entities=[drive_base_change_mode_to_FAST]))

        description = launch.LaunchDescription()
        description.add_action(on_inactive_handler)
        #description.add_action(on_active_handler)
        #description.add_action(on_FAST_mode)
        description.add_action(mode_manager)
        description.add_action(drive_base)
        description.add_action(manipulator)
        description.add_action(drive_base_configure)
        #description.add_action(drive_base_change_mode_to_FAST)
    except TypeError as err:
        print(err)
        print(sys.exc_type)
    except NameError as err:
        print(err)
        print(sys.exc_type)
    except AttributeError as err:
        print(Attributerr)
        print(sys.exc_type)
    except Exception as err:
        exception_type = type(err).__name__
        print('--Exception of type: ' + exception_type)
    else:
        print('all good, no exceptions')
    finally:
        print('finally')

    return description
