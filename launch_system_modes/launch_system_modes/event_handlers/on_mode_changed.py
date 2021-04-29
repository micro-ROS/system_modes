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

"""Module for OnModeChanged class."""

from typing import Callable
from typing import Optional
from typing import Text

from launch.event import Event
from launch.event_handler import EventHandler
from launch.some_actions_type import SomeActionsType
from launch.some_substitutions_type import SomeSubstitutionsType

from ..actions import SystemPart
from ..events import ModeChanged


class OnModeChanged(EventHandler):
    """Convenience class for handling a mode change of a system part."""

    def __init__(
        self,
        *,
        entities: SomeActionsType,
        target_system_part: SystemPart = None,
        start_mode: Optional[SomeSubstitutionsType] = None,
        goal_mode: Optional[SomeSubstitutionsType] = None,
        matcher: Optional[Callable[[Event], bool]] = None,
        **kwargs
    ) -> None:
        """
        Create an OnModeChanged event handler.

        There are several matching options, each of which is compared with the
        event and must match it to have the handler handle the event.
        Passing None for any of them will prevent that matching option from
        being considered (and therefore not required) when matching the event.
        If matcher is given, the other conditions are not considered.
        """
        if not isinstance(target_system_part, (SystemPart, type(None))):
            raise RuntimeError("OnModeChanged requires a 'SystemPart' action as the target")
        # Handle optional matcher argument.
        self.__custom_matcher = matcher
        if self.__custom_matcher is None:
            self.__custom_matcher = (
                lambda event: (
                    isinstance(event, ModeChanged) and (
                        target_system_part is None or
                        event.action == target_system_part
                    ) and (
                        start_mode is None or
                        event.start_mode == start_mode
                    ) and (
                        goal_mode is None or
                        event.goal_mode == goal_mode
                    )
                )
            )
        # Call parent init.
        super().__init__(
            matcher=self.__custom_matcher,
            entities=entities,
            **kwargs
        )
        self.__target_system_part = target_system_part
        print('OnModeChanged event handler initialized')

    @property
    def handler_description(self) -> Text:
        """Return the string description of the handler."""
        return '<actions>'

    @property
    def matcher_description(self):
        """Return the string description of the matcher."""
        if self.__target_system_part is None:
            print('OUIUDI')
            return 'event == ModeChanged'
        print('OUIUsdfDI')
        return 'event == ModeChanged and event.action == SystemPart({})'.format(
            hex(id(self.__target_system_part))
        )
