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

"""Tests for the Mode Changed event."""

import launch

from launch_system_modes.actions import Node, System
from launch_system_modes.events import ModeChanged

import pytest
import system_modes_msgs


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

    mode_event = system_modes_msgs.msg.ModeEvent()
    mode_event.start_mode.label = '__DEFAULT__'
    mode_event.goal_mode.label = 'FAST'

    # create mode changed event
    ModeChanged(
        action=drive_base,
        msg=mode_event,
    )
    mc = ModeChanged(
        action=actuation,
        msg=mode_event,
    )
    assert mc.start_mode == '__DEFAULT__'
    assert mc.goal_mode == 'FAST'

    # Construction with missing mandatory parameters
    with pytest.raises(TypeError):
        ModeChanged(
            action=drive_base,
        )
    with pytest.raises(TypeError):
        ModeChanged(
            msg=mode_event,
        )

    # try emitting / integration with launch actions
    launch.actions.EmitEvent(
        event=ModeChanged(
            action=actuation,
            msg=mode_event))
