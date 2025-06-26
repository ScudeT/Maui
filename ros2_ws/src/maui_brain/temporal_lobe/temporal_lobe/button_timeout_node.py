import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


class ToggleButtonNode(Node):
    def __init__(self):
        super().__init__('toggle_button_node')
        # Declare and read the timeout parameter (in seconds)
        self.declare_parameter('time_out', 30.0)
        self.time_out = self.get_parameter('time_out').get_parameter_value().double_value

        # Publisher for the button_state topic
        self.publisher_ = self.create_publisher(Bool, 'button_state', 10)
        # Service to toggle the node on/off
        self.srv = self.create_service(Trigger, 'trigger', self.trigger_callback)

        # Internal state
        self.is_on = False
        self._timer = None

    def trigger_callback(self, request, response):
        if not self.is_on:
            self.get_logger().info('Turning ON. Will publish after {:.1f}s'.format(self.time_out))
            self.is_on = True
            # Schedule a one-shot timer
            self._timer = self.create_timer(self.time_out, self._timeout_callback)
            response.success = True
            response.message = f'Activated: will publish in {self.time_out} seconds.'
        else:
            self.get_logger().info('Turning OFF. Cancelling pending publish.')
            self.is_on = False
            # Cancel the pending timer if it exists
            if self._timer:
                self._timer.cancel()
                self._timer = None
            response.success = True
            response.message = 'Deactivated: pending publish cancelled.'
        return response

    def _timeout_callback(self):
        # Only publish if still on
        if self.is_on:
            msg = Bool()
            msg.data = True
            self.publisher_.publish(msg)
            self.get_logger().info('Published True on button_state')
        # Clean up: one-shot behavior and reset state
        if self._timer:
            self._timer.cancel()
            self._timer = None


def main(args=None):
    rclpy.init(args=args)
    node = ToggleButtonNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
