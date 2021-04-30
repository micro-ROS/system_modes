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

"""Module for ChangeState event."""

from typing import Callable

from launch.event import Event

if False:
    from ..actions import System  # noqa: F401


class ChangeState(Event):
    """Event emitted when a state change is requested for a *system*."""

    name = 'launch_system_modes.events.ChangeState'

    def __init__(
        self,
        *,
        system_part_matcher: Callable[['System'], bool],
        transition_id: int
    ) -> None:
        """
        Create a ChangeState event.

        :param: system_part_matcher is a callable which returns True if the
            given lifecycle node should be affected by this event.
        :param: transition_id is the name of the requested mode
        """
        super().__init__()
        self.__system_part_matcher = system_part_matcher
        self.__transition_id = transition_id
        if transition_id == 1:
            print('ChangeState event initialized for transition: configure')
        elif transition_id == 3:
            print('ChangeState event initialized for transition: activate')
        else:
            print('ChangeState event initialized for transition: ' + str(transition_id))

    @property
    def system_part_matcher(self) -> Callable[['System'], bool]:
        """Getter for system_part_matcher."""
        return self.__system_part_matcher

    @property
    def transition_id(self) -> int:
        """Getter for transition_id."""
        return self.__transition_id
