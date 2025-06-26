#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import Trigger
from geometry_msgs.msg import Vector3

class SimpleCommandNode(Node):
    def __init__(self):
        super().__init__('simple_command_node')

        #–– Declare & read parameters ––#
        self.declare_parameter('delay', 1.0)
        self.declare_parameter('command', [0.0, 0.0, 0.0])
        self.delay = self.get_parameter('delay').get_parameter_value().double_value
        cmd = self.get_parameter('command').get_parameter_value().double_array_value
        if len(cmd) != 3:
            self.get_logger().fatal("Parameter 'command' must be an array of 3 floats")
            rclpy.shutdown()
            return
        self.command = cmd

        #–– Publisher ––#
        self.publisher_ = self.create_publisher(Vector3, 'input', 10)

        #–– Service ––#
        cb_group = ReentrantCallbackGroup()
        self.srv = self.create_service(
            Trigger, 'trigger', self.trigger_callback, callback_group=cb_group
        )

        #–– Dynamic parameter update timer (1 Hz) ––#
        self._param_timer = self.create_timer(
            1.0,
            self._update_parameters
        )

        #–– State ––#
        self.is_on = False
        self._timer = None

        self.get_logger().info(f"Ready. Delay={self.delay}s, command={self.command}")

    def _update_parameters(self):
        # Periodically refresh parameters from server
        new_delay = self.get_parameter('delay').get_parameter_value().double_value
        new_cmd = self.get_parameter('command').get_parameter_value().double_array_value
        if new_delay != self.delay:
            self.get_logger().info(f"Parameter 'delay' updated: {self.delay} -> {new_delay}")
            self.delay = new_delay
            # No need to restart existing timers; new delay will apply on next ON
        if len(new_cmd) == 3 and new_cmd != self.command:
            self.get_logger().info(f"Parameter 'command' updated: {self.command} -> {new_cmd}")
            self.command = new_cmd
        elif len(new_cmd) != 3:
            self.get_logger().error("Parameter 'command' must be an array of 3 floats; update ignored.")

    def trigger_callback(self, request, response):
        if self.is_on:
            # Turn off: publish zeros immediately
            self._cancel_timer()
            self._publish([0.0, 0.0, 0.0])
            self.is_on = False
            response.success = True
            response.message = 'Turned OFF: zeros sent'
            self.get_logger().info(response.message)
        else:
            # Turn on: first send zeros, then schedule actual command
            self._publish([0.0, 0.0, 0.0])
            self._timer = self.create_timer(
                self.delay,
                self._publish_command,
                callback_group=self.srv.callback_group
            )
            self.is_on = True
            response.success = True
            response.message = f"Turned ON: zeros sent, will send command after {self.delay}s"
            self.get_logger().info(response.message)
        return response

    def _publish_command(self):
        # Publish the actual command values then cancel the timer
        self._publish(self.command)
        self.get_logger().info(f"Published command: {self.command}")
        self._cancel_timer()

    def _cancel_timer(self):
        if self._timer:
            self._timer.cancel()
            self._timer = None

    def _publish(self, values):
        msg = Vector3()
        msg.x, msg.y, msg.z = values
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleCommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
