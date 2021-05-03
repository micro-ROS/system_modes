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

"""Tests for the Mode State event."""

import launch

from launch_system_modes.actions import System
from launch_system_modes.events import ChangeState

import pytest


def test_system_part_construction():
    actuation = System(
        name='actuation',
        namespace='')

    # create change mode
    ChangeState(
        system_part_matcher=launch.events.matchers.matches_action(actuation),
        transition_id=0,
    )

    # Construction with missing mandatory parameters
    with pytest.raises(TypeError):
        ChangeState(
            system_part_matcher=launch.events.matchers.matches_action(actuation)
        )
    with pytest.raises(TypeError):
        ChangeState(
            transition_id=0,
        )

    # try emitting / integration with launch actions
    launch.actions.EmitEvent(
        event=ChangeState(
            system_part_matcher=launch.events.matchers.matches_action(actuation),
            transition_id=0))
