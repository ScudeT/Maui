#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import Trigger
from std_msgs.msg import Float32

class MultiCommandNode(Node):

    def __init__(self):
        super().__init__('multi_command_node')

        #–– Declare & read parameters ––#
        self.declare_parameter('delay', 1.0)
        self.declare_parameter('commands', ['0.0,0.0,0.0,0.0,0.0,0.0:0.0,0.0,0.0,0.0,0.0,0.0'])
        self.delay = self.get_parameter('delay').get_parameter_value().double_value
        raw_cmds = self.get_parameter('commands').get_parameter_value().string_array_value

        # Parse "A,B,C,D,E,F:X,Y,Z,..." strings into [ ([A,...,F], [X,...]), ... ]
        self.commands = []
        for s in raw_cmds:
            try:
                first, second = s.split(':')
                a = [float(x) for x in first.split(',')]
                b = [float(x) for x in second.split(',')]
                if len(a)!=6 or len(b)!=6:
                    raise ValueError
                self.commands.append((a, b))
            except Exception:
                self.get_logger().error(f"Invalid command string: '{s}' (must be 6 floats, colon, then 6 floats)")
        if not self.commands:
            self.get_logger().fatal("No valid commands loaded; shutting down.")
            rclpy.shutdown()
            return

        #–– Publishers ––#
        self.publishers_ = []
        for i in range(6):
            topic = f'input{i+1}'
            pub = self.create_publisher(Float32, topic, 10)
            self.publishers_.append(pub)

        #–– Service ––#
        # Use Reentrant so timer callback & service callback can run concurrently if needed
        cb_group = ReentrantCallbackGroup()
        self.srv = self.create_service(
            Trigger, 'trigger', self.trigger_callback, callback_group=cb_group
        )

        #–– State ––#
        self.is_on = False
        self.current_index = 0
        self._second_timer = None

        self.get_logger().info(f"Ready. Delay={self.delay}s, {len(self.commands)} command-pairs loaded.")

    def trigger_callback(self, request, response):
        if self.is_on:
            # turn off
            self._cancel_pending_timer()
            self._publish_values([0.0]*6)
            self.is_on = False
            response.success = True
            response.message = 'Turned OFF (zeros sent)'
            self.get_logger().info(response.message)
        else:
            # turn on: send next command pair
            a, b = self.commands[self.current_index]
            self._publish_values(a)
            # schedule second
            self._second_timer = self.create_timer(
                self.delay, lambda: self._second_step(b), callback_group=self._timer_cb_group()
            )
            # advance index
            self.current_index = (self.current_index + 1) % len(self.commands)
            self.is_on = True
            response.success = True
            response.message = f"Turned ON: first values sent (will send second after {self.delay}s)"
            self.get_logger().info(response.message)
        return response

    def _second_step(self, values):
        # publish the second set, then destroy this one-shot timer
        self._publish_values(values)
        self.get_logger().info(f"Second values sent for command pair #{(self.current_index-1)%len(self.commands)}")
        if self._second_timer is not None:
            self._second_timer.cancel()
            self._second_timer = None

    def _cancel_pending_timer(self):
        if self._second_timer is not None:
            self._second_timer.cancel()
            self._second_timer = None

    def _publish_values(self, values):
        msg = Float32()
        for val, pub in zip(values, self.publishers_):
            msg.data = float(val)
            pub.publish(msg)

    def _timer_cb_group(self):
        # helper to reuse the same callback group as the service
        return self.srv.callback_group

def main(args=None):
    rclpy.init(args=args)
    node = MultiCommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
