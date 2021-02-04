import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState


class FakeLifecycleNode(Node):

    def __init__(self, name):
        super().__init__(name)
        
        self.declare_parameter("foo")
        self.declare_parameter("bar")

        self.srv = self.create_service(
            ChangeState,
            self.get_name() + '/change_state',
            self.change_state_callback)

    def parameter_callback(self, params):
        print(self.get_name() + " got parameters")
        for param in params:
            if param.name == 'bar' and param.type_ == Parameter.Type.STRING:
                print(self.name + " set parameter " + param.name + " to " + param.value)
            if param.name == 'foo' and param.type_ == Parameter.Type.DOUBLE:
                print(self.name + " set parameter " + param.name + " to " + param.value)
        return SetParametersResult(successful=True)

    def change_state_callback(self, request, response):
        response.success = True
        self.get_logger().info(
            'Incoming change state request for %s: %s' % (self.get_name(), request.transition.label))

        return response

class LifecycleClient(Node):

    def __init__(self):
        super().__init__("lifecycle_client")
        self.cli = self.create_client(ChangeState, '/sys/change_state')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = ChangeState.Request()

    def configure_system(self):
        self.req.transition.id = 1
        self.req.transition.label = 'configure'
        self.future = self.cli.call_async(self.req)

    def activate_system(self):
        self.req.transition.id = 3
        self.req.transition.label = 'activate'
        self.future = self.cli.call_async(self.req)

def main(args=None):
    rclpy.init(args=args)
    try:
        executor = MultiThreadedExecutor()

        node_a = FakeLifecycleNode("A")
        node_b = FakeLifecycleNode("B")
        node_c = FakeLifecycleNode("C")
        node_d = FakeLifecycleNode("D")

        executor.add_node(node_a)
        executor.add_node(node_b)
        executor.add_node(node_c)
        executor.add_node(node_d)

        lc = LifecycleClient()

        try:
            lc.configure_system()

            executor.spin_once(timeout_sec=1)
            executor.spin_once(timeout_sec=1)
            executor.spin_once(timeout_sec=1)

            lc.activate_system()

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
