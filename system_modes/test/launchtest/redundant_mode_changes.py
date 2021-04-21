from time import sleep

from lifecycle_msgs.srv import ChangeState
from rcl_interfaces.msg import SetParametersResult

import rclpy

from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter

from system_modes_msgs.srv import ChangeMode


class FakeLifecycleNode(Node):
    num_param_callbacks = 0

    def __init__(self, name):
        super().__init__(name)

        self.declare_parameter('foo', 0.0)
        self.declare_parameter('bar', 'ZERO')
        self.add_on_set_parameters_callback(self.parameter_callback)

        # State change service
        self.srv = self.create_service(
            ChangeState,
            self.get_name() + '/change_state',
            self.change_state_callback)

    def parameter_callback(self, params):
        for p in params:
            if p.name == 'bar' and p.type_ == Parameter.Type.STRING:
                self.get_logger().info('Parameter callback #%d %s:%s:%s'
                                       % (self.num_param_callbacks,
                                          self.get_name(),
                                          p.name, p.value))
            if p.name == 'foo' and p.type_ == Parameter.Type.DOUBLE:
                self.get_logger().info('Parameter callback #%d %s:%s:%s'
                                       % (self.num_param_callbacks,
                                          self.get_name(),
                                          p.name, p.value))
        self.num_param_callbacks = self.num_param_callbacks + 1
        return SetParametersResult(successful=True)

    def change_state_callback(self, request, response):
        response.success = True
        self.get_logger().info('Transition %s:%s' % (self.get_name(), request.transition.label))

        return response


class LifecycleClient(Node):

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

        self.climn = self.create_client(ChangeMode, '/A/change_mode')
        while not self.clim.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.reqmn = ChangeMode.Request()

    def configure_system(self):
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

    def change_A_mode(self, mode):
        self.reqmn.mode_name = mode
        self.future = self.climn.call_async(self.reqmn)


def main(args=None):
    rclpy.init(args=args)
    try:
        executor = MultiThreadedExecutor()
        node_a = FakeLifecycleNode('A')
        node_b = FakeLifecycleNode('B')

        executor.add_node(node_a)
        executor.add_node(node_b)

        lc = LifecycleClient()

        try:
            lc.configure_system()
            executor.spin_once(timeout_sec=1)
            executor.spin_once(timeout_sec=1)
            sleep(2)

            lc.activate_system()
            executor.spin_once(timeout_sec=1)
            executor.spin_once(timeout_sec=1)
            executor.spin_once(timeout_sec=1)
            sleep(2)  # give the system some time to converge

            lc.change_A_mode('AA')
            executor.spin_once(timeout_sec=1)
            executor.spin_once(timeout_sec=1)
            executor.spin_once(timeout_sec=1)
            executor.spin_once(timeout_sec=1)
            executor.spin_once(timeout_sec=1)
            sleep(3)  # give the system some time to converge

            lc.change_A_mode('AA')  # this is the tested aspect: call redundant, should be ignored
            executor.spin_once(timeout_sec=1)
            executor.spin_once(timeout_sec=1)
            sleep(2)  # give the system some time to converge

            lc.change_A_mode('BB')
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
