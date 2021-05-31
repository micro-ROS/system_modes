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

"""Module for StateTransition event."""

from typing import Text

from launch.event import Event

import lifecycle_msgs.msg

if False:
    from ..actions import System  # noqa: F401


class StateTransition(Event):
    """Event emitted when a state changed."""

    name = 'launch_system_modes.events.StateTransition'

    def __init__(
        self,
        *,
        action: 'System',
        msg: lifecycle_msgs.msg.TransitionEvent
    ) -> None:
        """
        Create a StateTransition event.

        :param: action the instance of class::`System` that generated this event
        :param: msg the instance of the ROS message TransitionEvent that generated this event
        """
        super().__init__()
        self.__action = action
        self.__msg = msg
        self.__transition = msg.transition.label
        self.__start_state = msg.start_state.label
        self.__goal_state = msg.goal_state.label
        print('StateTransition event initialized for: ' + self.__transition)

    @property
    def action(self) -> 'System':
        """Getter for action."""
        return self.__action

    @property
    def msg(self) -> lifecycle_msgs.msg.TransitionEvent:
        """Getter for msg."""
        return self.__msg

    @property
    def transition(self) -> Text:
        """Getter for the transition."""
        return self.__transition

    @property
    def start_state(self) -> Text:
        """Getter for the start state."""
        return self.__start_state

    @property
    def goal_state(self) -> Text:
        """Getter for the goal state."""
        return self.__goal_state
