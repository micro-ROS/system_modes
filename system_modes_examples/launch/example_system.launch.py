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
import launch.launch_description_sources
import launch.substitutions


def generate_launch_description():
    modelfile = (ament_index_python.packages.get_package_share_directory('system_modes_examples') +
                 '/example_modes.yaml')

    mode_manager = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            ament_index_python.packages.get_package_share_directory(
                'system_modes') + '/launch/mode_manager.launch.py'),
        launch_arguments={'modelfile': modelfile}.items())

    drive_base = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            ament_index_python.packages.get_package_share_directory(
                'system_modes_examples') + '/launch/drive_base.launch.py'),
        launch_arguments={'modelfile': modelfile}.items())

    manipulator = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            ament_index_python.packages.get_package_share_directory(
                'system_modes_examples') + '/launch/manipulator.launch.py'),
        launch_arguments={'modelfile': modelfile}.items())

    description = launch.LaunchDescription()
    description.add_action(mode_manager)
    description.add_action(drive_base)
    description.add_action(manipulator)

    return description
