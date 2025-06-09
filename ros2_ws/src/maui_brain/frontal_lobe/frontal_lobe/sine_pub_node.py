import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_srvs.srv import Trigger
import math
import time


class SinePublisher(Node):
    def __init__(self):
        super().__init__('sine_publisher')

        # === Dichiarazione Parametri ===
        self.declare_parameter('freq', 10.0)
        self.declare_parameter('amplitude', 1.0)
        self.declare_parameter('period', 2.0)
        self.declare_parameter('offset', 0.0)
        self.declare_parameter('start_from', 0.0)
        self.declare_parameter('delay', 0.0)
        self.declare_parameter('upper_limit', 1.0)
        self.declare_parameter('lower_limit', -1.0)
        self.declare_parameter('topic_name', 'sine_value')

        # === Lettura Parametri ===
        self.freq = self.get_parameter('freq').value
        self.amplitude = self.get_parameter('amplitude').value
        self.period = self.get_parameter('period').value
        self.offset = self.get_parameter('offset').value
        self.start_from_value = self.get_parameter('start_from').value
        self.delay = self.get_parameter('delay').value
        self.upper_limit = self.get_parameter('upper_limit').value
        self.lower_limit = self.get_parameter('lower_limit').value

        # === Calcolo fase iniziale dalla start_from_value ===
        if self.amplitude == 0.0:
            self.phase = 0.0
        else:
            ratio = (self.start_from_value - self.offset) / self.amplitude
            ratio_clamped = min(max(ratio, -1.0), 1.0)
            self.phase = math.asin(ratio_clamped)

        # === Publisher e Timer ===
        self.publisher_ = self.create_publisher(Float32, "sin", 10)
        self.timer = self.create_timer(1.0 / self.freq, self.publish_value)

        # === Stato ON/OFF e tempo ===
        self.active = False
        self.activation_time = None  # Set when triggered ON

        # === Servizio Trigger ===
        self.srv = self.create_service(Trigger, 'toggle_sinusoid', self.handle_trigger)

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
                # Delay non ancora trascorso → non pubblica nulla
                return
            t = elapsed - self.delay
            value = self.amplitude * math.sin(2 * math.pi * t / self.period + self.phase) + self.offset
            value = min(max(value, self.lower_limit), self.upper_limit)
            msg.data = value
        else:
            msg.data = self.start_from_value  # Stato OFF

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SinePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

