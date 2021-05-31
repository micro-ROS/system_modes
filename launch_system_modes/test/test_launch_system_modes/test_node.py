# Copyright 2021 Robert Bosch GmbH
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

"""Tests for the Node Action."""

from launch import LaunchContext
from launch_system_modes.actions import Node

import pytest


def test_system_part_construction():
    Node(
        package='system_modes_examples',
        executable='drive_base',
        name='drive_base',
        namespace='',
        output='screen')

    # Construction with missing mandatory parameters
    with pytest.raises(TypeError):
        Node(
            package='system_modes_examples',
            name='drive_base',
            namespace='',
            output='screen')
    with pytest.raises(TypeError):
        Node(
            package='system_modes_examples',
            executable='drive_base',
            namespace='',
            output='screen')
    with pytest.raises(TypeError):
        Node(
            package='system_modes_examples',
            executable='drive_base',
            name='drive_base',
            output='screen')


def test_node_name():
    part = Node(
        package='system_modes_examples',
        executable='drive_base',
        name='drive_base',
        namespace='testtest',
    )
    lc = LaunchContext()
    part._perform_substitutions(lc)
    assert part.is_node_name_fully_specified() is True
