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

"""Module for ModeChanged event."""

from typing import Text

from launch.event import Event

import system_modes_msgs.msg

if False:
    from ..actions import SystemPart  # noqa: F401


class ModeChanged(Event):
    """Event emitted when a system mode changed."""

    name = 'launch_ros.events.ModeChanged'

    def __init__(
        self,
        *,
        action: 'SystemPart',
        msg: system_modes_msgs.msg.ModeEvent
    ) -> None:
        """
        Create a ModeChanged event.

        :param: action the instance of class::`SystemPart` that generated this event
        :param: msg the instance of the ROS message ModeEvent that generated this event
        """
        super().__init__()
        self.__action = action
        self.__msg = msg
        self.__timestamp = msg.timestamp
        self.__start_mode = msg.start_mode.label
        self.__goal_mode = msg.goal_mode.label
        print('ModeChanged event created: ' + self.__action.get_name() + ' changed to '
              + self.__goal_mode)

    @property
    def action(self) -> 'SystemPart':
        """Getter for action."""
        return self.__action

    @property
    def msg(self) -> system_modes_msgs.msg.ModeEvent:
        """Getter for msg."""
        return self.__msg

    @property
    def timestamp(self) -> int:
        """Getter for timestamp."""
        return self.__timestamp

    @property
    def start_mode(self) -> Text:
        """Getter for start_mode."""
        return self.__start_mode

    @property
    def goal_mode(self) -> Text:
        """Getter for goal_mode."""
        return self.__goal_mode
