import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_srvs.srv import Trigger
import math
import time


class SinePublisher(Node):
    def __init__(self):
        super().__init__('sine_publisher')

        # === Declare parameters with defaults ===
        self.declare_parameter('freq', 10.0)
        self.declare_parameter('amplitude', 1.0)
        self.declare_parameter('period', 2.0)
        self.declare_parameter('offset', 0.0)
        self.declare_parameter('start_from', 0.0)
        self.declare_parameter('delay', 0.0)
        self.declare_parameter('upper_limit', 1.0)
        self.declare_parameter('lower_limit', -1.0)
        self.declare_parameter('topic_name', 'sine_value')

        # === Read them once at startup ===
        self._read_parameters()

        # === Compute initial phase ===
        self._compute_initial_phase()

        # === Publisher & main-timer (triggered at 'freq') ===
        self.publisher_ = self.create_publisher(
            Float32,
            self.get_parameter('topic_name').value,
            10
        )
        self.sine_timer = self.create_timer(
            1.0 / self.freq,
            self.publish_value
        )

        # === Parameter-watcher timer (fires once per second) ===
        self.param_timer = self.create_timer(
            1.0,
            self.update_parameters
        )

        # === ON/OFF state + Trigger service ===
        self.active = False
        self.activation_time = None
        self.srv = self.create_service(
            Trigger,
            'toggle_sinusoid',
            self.handle_trigger
        )

    def _read_parameters(self):
        p = self.get_parameter
        self.freq          = p('freq').value
        self.amplitude     = p('amplitude').value
        self.period        = p('period').value
        self.offset        = p('offset').value
        self.start_from_value = p('start_from').value
        self.delay         = p('delay').value
        self.upper_limit   = p('upper_limit').value
        self.lower_limit   = p('lower_limit').value

    def _compute_initial_phase(self):
        if self.amplitude == 0.0:
            self.phase = 0.0
        else:
            ratio = (self.start_from_value - self.offset) / self.amplitude
            ratio_clamped = min(max(ratio, -1.0), 1.0)
            self.phase = math.asin(ratio_clamped)

    def update_parameters(self):
        """
        Called once per second.  Re-fetch all parameters,
        and if 'freq' changed, re-create the sine_timer
        so it runs at the new rate.
        """
        old_freq = self.freq
        self._read_parameters()

        # If freq changed, rebuild the sine_timer with new period
        if self.freq != old_freq:
            # cancel old timer
            self.sine_timer.cancel()
            # create a new one at the new rate
            self.sine_timer = self.create_timer(
                1.0 / self.freq,
                self.publish_value
            )
            self.get_logger().info(f"freq changed → new publish rate: {self.freq} Hz")

        # If amplitude, offset, or start_from changed, recompute phase
        # (so it picks up a new start value immediately)
        # You could add similar checks/logs for other params if you like:
        self._compute_initial_phase()

    def handle_trigger(self, request, response):
        self.active = not self.active
        if self.active:
            self.activation_time = time.time()
            response.message = "Sinusoid turned ON"
        else:
            self.activation_time = None
            response.message = "Sinusoid turned OFF"
        response.success = True
        self.get_logger().info(response.message)
        return response

    def publish_value(self):
        msg = Float32()
        if self.active:
            elapsed = time.time() - self.activation_time
            if elapsed < self.delay:
                return  # still in startup delay
            t = elapsed - self.delay
            val = self.amplitude * math.cos(
                2 * math.pi * t / self.period + self.phase
            ) + self.offset
            # clamp to limits
            val = min(max(val, self.lower_limit), self.upper_limit)
            msg.data = val
        else:
            msg.data = self.start_from_value
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SinePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

