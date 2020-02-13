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

import ament_index_python.packages

import launch
import launch.actions
import launch.launch_description_sources
import launch.substitutions
import launch_ros


def generate_launch_description():
    launch.actions.DeclareLaunchArgument('modelfile', description='Path to modelfile')
    modelfile = (ament_index_python.packages.get_package_share_directory('system_modes_examples') +
                '/example_modes.yaml')

    node = launch_ros.actions.Node(
        package='system_modes_examples',
        node_executable='manipulator',
        parameters=[{'modelfile': modelfile}],
        output='screen')

    description = launch.LaunchDescription()
    description.add_action(node)

    return description