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

"""Tests for the State Transition event."""

import launch

from launch_system_modes.actions import System
from launch_system_modes.events import StateTransition

import lifecycle_msgs
import pytest


def test_system_part_construction():
    actuation = System(
        name='actuation',
        namespace='')

    transition_msg = lifecycle_msgs.msg.TransitionEvent()
    transition_msg.start_state.label = 'inactive'
    transition_msg.goal_state.label = 'active'

    # create state trasition event
    st = StateTransition(
        action=actuation,
        msg=transition_msg
    )
    assert st.start_state == 'inactive'
    assert st.goal_state == 'active'

    # Construction with missing mandatory parameters
    with pytest.raises(TypeError):
        StateTransition(
            action=actuation,
        )
    with pytest.raises(TypeError):
        StateTransition(
            msg=transition_msg
        )

    # try emitting / integration with launch actions
    launch.actions.EmitEvent(
        event=StateTransition(
            action=actuation,
            msg=transition_msg))
