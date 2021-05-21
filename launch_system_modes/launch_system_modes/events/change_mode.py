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

"""Module for ChangeMode event."""

from typing import Callable

from launch.event import Event

if False:
    from launch_system_modes.actions import SystemPart  # noqa: F401


class ChangeMode(Event):
    """Event emitted when a system mode change is requested for a system or node."""

    name = 'launch_system_modes.events.ChangeMode'

    def __init__(
        self,
        *,
        system_part_matcher: Callable[['SystemPart'], bool],
        mode_name: str
    ) -> None:
        """
        Create a ChangeMode event.

        :param: system_part_matcher is a callable which returns True if the
            given system part should be affected by this event.
        :param: mode_name is the name of the requested mode
        """
        super().__init__()
        self.__system_part_matcher = system_part_matcher
        self.__mode_name = mode_name
        print('ChangeMode event initialized for mode: '+mode_name)

    @property
    def system_part_matcher(self) -> Callable[['SystemPart'], bool]:
        """Getter for system_part_matcher."""
        return self.__system_part_matcher

    @property
    def mode_name(self) -> int:
        """Getter for mode_name."""
        return self.__mode_name
