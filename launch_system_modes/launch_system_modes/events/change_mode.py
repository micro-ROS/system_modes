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

from launch_ros.actions import LifecycleNode  # noqa: F401


class ChangeMode(Event):
    """Event emitted when a system mode transition is requested for a system or node."""

    name = 'launch_system_modes.events.ChangeMode'

    def __init__(
        self,
        *,
        lifecycle_node_matcher: Callable[['LifecycleNode'], bool],
        mode_name: str
    ) -> None:
        """
        Create a ChangeMode event.

        :param: lifecycle_node_matcher is a callable which returns True if the
            given lifecycle node should be affected by this event.
        :param: mode_name is the name of the requested mode
        """
        super().__init__()
        self.__lifecycle_node_matcher = lifecycle_node_matcher
        self.__mode_name = mode_name
        print('ChangeMode event initialized')

    @property
    def lifecycle_node_matcher(self) -> Callable[['LifecycleNode'], bool]:
        """Getter for lifecycle_node_matcher."""
        return self.__lifecycle_node_matcher

    @property
    def mode_name(self) -> int:
        """Getter for mode_name."""
        return self.__mode_name
