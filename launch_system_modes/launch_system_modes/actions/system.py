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

"""Module for the system modes actions."""

import functools
import threading
from typing import cast
from typing import List
from typing import Optional

import launch
from launch import SomeSubstitutionsType
from launch.action import Action
import launch.logging
from launch_ros.ros_adapters import get_ros_node

import lifecycle_msgs.msg
import system_modes_msgs.msg
import system_modes_msgs.srv

from .system_part import SystemPart
from ..events import ChangeMode
from ..events import ChangeState
from ..events import ModeChanged
from ..events import StateTransition


class System(Action, SystemPart):
    """Action that handles system modes of systems."""

    def __init__(
        self,
        *,
        name: SomeSubstitutionsType,
        namespace: SomeSubstitutionsType,
        **kwargs
    ) -> None:
        """
        Construct a System action.

        :param name: The name of the system.
        :param namespace: The namespace of the system.
        """
        Action.__init__(self, **kwargs)
        SystemPart.__init__(self, name, namespace)

        self.node_name = self.get_name()  # todo: get rid of node_name

        self.__logger = launch.logging.get_logger(__name__)
        self.__rclpy_mode_subscription = None
        self.__rclpy_state_subscription = None
        print('System action "' + self.get_name() + '" initialized')

    def _on_mode_event(self, context, msg):
        print('System action "' + self.get_name() + '" caught mode event ' + msg.goal_mode.label)

        event = ModeChanged(action=self, msg=msg)
        self.__current_mode = msg.goal_mode.label
        print(' -> emitting ModeChanged event for ' + self.get_name() + 's change to '
              + msg.goal_mode.label)
        context.asyncio_loop.call_soon_threadsafe(lambda: context.emit_event_sync(event))

    def _on_state_event(self, context, msg):
        print('System action "' + self.get_name() + '" caught transition event '
              + str(msg.transition.id))

        event = StateTransition(action=self, msg=msg)
        self.__current_state = msg.goal_state.label
        print(' -> emitting StateTransition event for ' + self.get_name() + 's change to '
              + msg.goal_state.label)
        context.asyncio_loop.call_soon_threadsafe(lambda: context.emit_event_sync(event))

    def _call_change_mode(self, request, context: launch.LaunchContext):
        print('System action "' + self.get_name() + '" _call_change_mode')
        while not self.__rclpy_change_mode_client.wait_for_service(timeout_sec=1.0):
            if context.is_shutdown:
                self.__logger.warning(
                    "Abandoning wait for the '{}' service, due to shutdown.".format(
                        self.__rclpy_change_mode_client.srv_name),
                )
                return

        # Asynchronously wait so that we can periodically check for shutdown.
        event = threading.Event()

        def unblock(future):
            nonlocal event
            event.set()

        response_future = self.__rclpy_change_mode_client.call_async(request)
        response_future.add_done_callback(unblock)

        while not event.wait(1.0):
            if context.is_shutdown:
                self.__logger.warning(
                    "Abandoning wait for the '{}' service response, due to shutdown.".format(
                        self.__rclpy_change_mode_client.srv_name),
                )
                response_future.cancel()
                return

        if response_future.exception() is not None:
            raise response_future.exception()
        response = response_future.result()

        if not response.success:
            self.__logger.error(
                "Failed to make transition '{}' for system part '{}'".format(
                    request.mode_name,
                    self.node_name,
                )
            )

    def _call_change_state(self, request, context: launch.LaunchContext):
        print('System action "' + self.get_name() + '" _call_change_state')
        while not self.__rclpy_change_state_client.wait_for_service(timeout_sec=1.0):
            if context.is_shutdown:
                self.__logger.warning(
                    "Abandoning wait for the '{}' service, due to shutdown.".format(
                        self.__rclpy_change_mode_client.srv_name),
                )
                return

        # Asynchronously wait so that we can periodically check for shutdown.
        event = threading.Event()

        def unblock(future):
            nonlocal event
            event.set()

        response_future = self.__rclpy_change_state_client.call_async(request)
        response_future.add_done_callback(unblock)

        while not event.wait(1.0):
            if context.is_shutdown:
                self.__logger.warning(
                    "Abandoning wait for the '{}' service response, due to shutdown.".format(
                        self.__rclpy_change_state_client.srv_name),
                )
                response_future.cancel()
                return

        if response_future.exception() is not None:
            raise response_future.exception()
        response = response_future.result()

        if not response.success:
            self.__logger.error(
                "Failed to make transition '{}' for system part '{}'".format(
                    request.transition.id,
                    self.node_name,
                )
            )

    def _on_change_mode_event(self, context: launch.LaunchContext) -> None:
        print('System action "' + self.get_name() + '" caught mode change request')
        typed_event = cast(ChangeMode, context.locals.event)
        if not typed_event.system_part_matcher(self):
            return None
        request = system_modes_msgs.srv.ChangeMode.Request()
        request.mode_name = typed_event.mode_name
        print(' -> attempting mode change of ' + self.get_name() + ' to ' + typed_event.mode_name)
        context.add_completion_future(
            context.asyncio_loop.run_in_executor(None, self._call_change_mode, request, context))

    def _on_change_state_event(self, context: launch.LaunchContext) -> None:
        print('System action "' + self.get_name() + '" caught state change request')
        typed_event = cast(ChangeState, context.locals.event)
        if not typed_event.system_part_matcher(self):
            return None
        request = lifecycle_msgs.srv.ChangeState.Request()
        request.transition.id = typed_event.transition_id
        print(' -> attempting state change of ' + self.get_name() + ' to '
              + str(typed_event.transition_id))
        context.add_completion_future(
            context.asyncio_loop.run_in_executor(None, self._call_change_state, request, context))

    def execute(self, context: launch.LaunchContext) -> Optional[List[Action]]:
        """
        Execute the action.

        Delegated to :meth:`launch.actions.ExecuteProcess.execute`.
        """
        # self._perform_substitutions(context)  # ensure self.node_name is expanded
        if '<node_name_unspecified>' in self.node_name:
            raise RuntimeError('node_name unexpectedly incomplete for system_modes node')
        node = get_ros_node(context)

        # Create a subscription to monitor the mode changes of the subprocess.
        self.__rclpy_mode_subscription = node.create_subscription(
            system_modes_msgs.msg.ModeEvent,
            '{}/mode_event'.format(self.node_name),
            functools.partial(self._on_mode_event, context),
            10)

        # Create a subscription to monitor the state changes of the subprocess.
        self.__rclpy_state_subscription = node.create_subscription(
            lifecycle_msgs.msg.TransitionEvent,
            '{}/transition_event'.format(self.node_name),
            functools.partial(self._on_state_event, context),
            10)

        # Create a service client to change mode on demand.
        self.__rclpy_change_mode_client = node.create_client(
            system_modes_msgs.srv.ChangeMode,
            '{}/change_mode'.format(self.node_name))

        # Create a service client to change state on demand.
        self.__rclpy_change_state_client = node.create_client(
            lifecycle_msgs.srv.ChangeState,
            '{}/change_state'.format(self.node_name))

        # Register an event handler to change modes on a ChangeMode system_modes event.
        context.register_event_handler(launch.EventHandler(
            matcher=lambda event: isinstance(event, ChangeMode),
            entities=[launch.actions.OpaqueFunction(function=self._on_change_mode_event)],
        ))

        # Register an event handler to change states on a ChangeState event.
        context.register_event_handler(launch.EventHandler(
            matcher=lambda event: isinstance(event, ChangeState),
            entities=[launch.actions.OpaqueFunction(function=self._on_change_state_event)],
        ))

        # Delegate execution to ExecuteProcess.
        return super().execute(context)
