from time import sleep

from lifecycle_msgs.srv import ChangeState
from rcl_interfaces.msg import SetParametersResult

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter

from system_modes_msgs.srv import ChangeMode


class FakeLifecycleNode(Node):

    def __init__(self, name):
        super().__init__(name)

        self.declare_parameter('foo')
        self.declare_parameter('bar')
        self.add_on_set_parameters_callback(self.parameter_callback)

        # State change service
        self.srv = self.create_service(
            ChangeState,
            self.get_name() + '/change_state',
            self.change_state_callback)

    def parameter_callback(self, params):
        for p in params:
            if p.name == 'bar' and p.type_ == Parameter.Type.STRING:
                self.get_logger().info('Parameter %s:%s:%s' % (self.get_name(), p.name, p.value))
            if p.name == 'foo' and p.type_ == Parameter.Type.DOUBLE:
                self.get_logger().info('Parameter %s:%s:%s' % (self.get_name(), p.name, p.value))
        return SetParametersResult(successful=True)

    def change_state_callback(self, request, response):
        response.success = True
        self.get_logger().info('Transition %s:%s' % (self.get_name(), request.transition.label))

        return response


class LifecycleClient(Node):

    def __init__(self, system_name):
        super().__init__('system_modes_test_client')

        self.clis = self.create_client(ChangeState, system_name + '/change_state')
        while not self.clis.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.reqs = ChangeState.Request()

        self.clim = self.create_client(ChangeMode, system_name + '/change_mode')
        while not self.clim.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.reqm = ChangeMode.Request()

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


def main(args=None):
    rclpy.init(args=args)
    try:
        executor = SingleThreadedExecutor()
        node_a = FakeLifecycleNode('A')
        node_b = FakeLifecycleNode('B')
        node_c = FakeLifecycleNode('C')
        node_d = FakeLifecycleNode('D')

        executor.add_node(node_a)
        executor.add_node(node_b)
        executor.add_node(node_c)
        executor.add_node(node_d)

        lc = LifecycleClient('sys')
        lc2 = LifecycleClient('sys2')

        try:
            lc.configure_system()
            executor.spin_once(timeout_sec=1)
            executor.spin_once(timeout_sec=1)
            lc2.configure_system()
            executor.spin_once(timeout_sec=1)
            executor.spin_once(timeout_sec=1)
            sleep(2)  # give the system some time to converge

            lc.activate_system()
            executor.spin_once(timeout_sec=1)
            executor.spin_once(timeout_sec=1)
            executor.spin_once(timeout_sec=1)
            lc2.activate_system()
            executor.spin_once(timeout_sec=1)
            executor.spin_once(timeout_sec=1)
            executor.spin_once(timeout_sec=1)
            sleep(2)  # give the system some time to converge

            lc.change_mode('CC')
            executor.spin_once(timeout_sec=1)
            executor.spin_once(timeout_sec=1)
            lc2.change_mode('DD')
            executor.spin()
        finally:
            executor.shutdown()
            node_a.destroy_node()
            node_b.destroy_node()
            node_c.destroy_node()
            node_d.destroy_node()
    finally:
        rclpy.shutdown()
        return 0


if __name__ == '__main__':
    main()
