from time import sleep

from lifecycle_msgs.msg import TransitionEvent
from lifecycle_msgs.srv import ChangeState
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter

from system_modes.msg import ModeEvent
from system_modes.srv import ChangeMode


class FakeLifecycleNode(Node):

    def __init__(self, name):
        super().__init__(name)

        self.declare_parameter('foo')
        self.declare_parameter('bar')

        self.pubs = self.create_publisher(
            TransitionEvent, self.get_name() + '/transition_event',
            10)
        self.pubm = self.create_publisher(ModeEvent, self.get_name() + '/mode_event', 10)

        # State change service
        self.srvs = self.create_service(
            ChangeState,
            self.get_name() + '/change_state',
            self.change_state_callback)
        self.add_on_set_parameters_callback(self.parameter_callback)

    def change_state_callback(self, request, response):
        response.success = True

        msg = TransitionEvent()
        if request.transition.id == 1 or request.transition.id == 4:
            print('node ', self.get_name(), ' pretending to go to "inactive"')
            msg.transition = request.transition
            msg.goal_state.id = 2
            msg.goal_state.label = 'inactive'
        elif request.transition.id == 3:
            print('node ', self.get_name(), ' pretending to go to "active"')
            msg.transition = request.transition
            msg.goal_state.id = 3
            msg.goal_state.label = 'active'
        self.pubs.publish(msg)

        return response

    def parameter_callback(self, params):
        for p in params:
            if p.name == 'foo' and p.type_ == Parameter.Type.DOUBLE:
                msg = ModeEvent()
                if p.value == 0.2:
                    msg.goal_mode.label = 'EE'
                elif p.value == 0.9:
                    msg.goal_mode.label = 'FF'
                else:
                    # default mode
                    msg.goal_mode.label = '__DEFAULT__'
                self.pubm.publish(msg)
        self.successful = True
        return self


class LifecycleClient(Node):

    ready = False

    def __init__(self):
        super().__init__('system_modes_test_client')

        self.clis = self.create_client(ChangeState, '/sys/change_state')
        while not self.clis.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.reqs = ChangeState.Request()

        self.clim = self.create_client(ChangeMode, '/sys/change_mode')
        while not self.clim.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.reqm = ChangeMode.Request()

        self.ready = True

    def configure_system(self):
        while not self.ready:
            sleep(.2)
        self.reqs.transition.id = 1
        self.reqs.transition.label = 'configure'
        self.future = self.clis.call_async(self.reqs)

    def activate_system(self):
        self.reqs.transition.id = 3
        self.reqs.transition.label = 'activate'
        self.future = self.clis.call_async(self.reqs)

    def change_mode(self, mode):
        self.reqm.mode_name = mode
        self.future = self.clim.call_async(self.reqm)


def main(args=None):
    rclpy.init(args=args)
    try:
        executor = SingleThreadedExecutor()
        node_a = FakeLifecycleNode('A')
        node_b = FakeLifecycleNode('B')

        executor.add_node(node_a)
        executor.add_node(node_b)

        lc = LifecycleClient()

        try:
            lc.configure_system()
            executor.spin_once(timeout_sec=1)
            executor.spin_once(timeout_sec=1)
            sleep(2)  # give the system some time to converge

            lc.activate_system()
            executor.spin_once(timeout_sec=1)
            executor.spin_once(timeout_sec=1)
            executor.spin_once(timeout_sec=1)
            sleep(2)  # give the system some time to converge

            lc.change_mode('CC')
            executor.spin()
        finally:
            executor.shutdown()
            node_a.destroy_node()
            node_b.destroy_node()
    finally:
        rclpy.shutdown()
        return 0


if __name__ == '__main__':
    main()
