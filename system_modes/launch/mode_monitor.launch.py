# Copyright (c) 2019 - for information on the respective copyright owner
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

import launch
import launch.actions

import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'modelfile',
            description='Path to modelfile'),
        launch.actions.DeclareLaunchArgument(
            'debug',
            default_value='false',
            description='Debug'),
        launch.actions.DeclareLaunchArgument(
            'verbose',
            default_value='false',
            description='Print mode parametrization'),
        launch.actions.DeclareLaunchArgument(
            'rate',
            default_value='1000',
            description='Monitor refresh rate in ms'),
        launch_ros.actions.Node(
            package='system_modes',
            node_executable='mode_monitor',
            parameters=[{
                'modelfile': launch.substitutions.LaunchConfiguration('modelfile'),
                'debug': launch.substitutions.LaunchConfiguration('debug'),
                'verbose': launch.substitutions.LaunchConfiguration('verbose'),
                'rate': launch.substitutions.LaunchConfiguration('rate')
                }],
            output='screen')])
