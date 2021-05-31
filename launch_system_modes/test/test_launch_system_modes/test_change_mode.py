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

"""Tests for the Mode Change event."""

import launch

from launch_system_modes.actions import Node, System
from launch_system_modes.events import ChangeMode

import pytest


def test_system_part_construction():
    drive_base = Node(
        package='system_modes_examples',
        executable='drive_base',
        name='drive_base',
        namespace='',
        output='screen')
    actuation = System(
        name='actuation',
        namespace='')

    # create change mode
    ChangeMode(
        system_part_matcher=launch.events.matchers.matches_action(drive_base),
        mode_name='__DEFAULT__',
    )
    ChangeMode(
        system_part_matcher=launch.events.matchers.matches_action(actuation),
        mode_name='__DEFAULT__',
    )

    # Construction with missing mandatory parameters
    with pytest.raises(TypeError):
        ChangeMode(
            system_part_matcher=launch.events.matchers.matches_action(drive_base)
        )
    with pytest.raises(TypeError):
        ChangeMode(
            mode_name='__DEFAULT__'
        )

    # try emitting / integration with launch actions
    launch.actions.EmitEvent(
        event=ChangeMode(
            system_part_matcher=launch.events.matchers.matches_action(actuation),
            mode_name='PERFORMANCE'))
