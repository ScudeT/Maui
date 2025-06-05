import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger
import math

class YawIntegratorNode(Node):

    def __init__(self):
        super().__init__('yaw_integrator_node')

        # Declare and get parameters
        self.declare_parameter('freq', 10)
        self.declare_parameter('wz', 0.1)
        self.frequency = self.get_parameter('freq').get_parameter_value().integer_value
        self.wz = self.get_parameter('wz').get_parameter_value().double_value

        # Publisher
        self.yaw_pub = self.create_publisher(Float32, 'yaw', 10)

        # Subscriber
        self.measure_sub = self.create_subscription(
            Odometry, 'measure', self.measure_callback, 10)

        # Trigger service
        self.trigger_srv = self.create_service(Trigger, 'trigger', self.trigger_callback)

        # Timer
        self.timer = self.create_timer(float(1.0 / self.frequency), self.timer_callback)

        # State
        self.running = False
        self.initialized = False
        self.current_yaw = 0.0
        self.last_time = self.get_clock().now()

        # Store the most recent measure yaw
        self.latest_measure_yaw = None

    def measure_callback(self, msg):
        q = msg.pose.pose.orientation
        yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)
        self.latest_measure_yaw = yaw

        # Initialize on first measure ever (legacy, can keep for robustness)
        if not self.initialized:
            self.current_yaw = yaw
            self.initialized = True
            self.get_logger().info(f'Initial yaw set from measure: {yaw:.3f} rad')

    def timer_callback(self):
        if not self.running or not self.initialized:
            return

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        self.current_yaw += self.wz * dt

        msg = Float32()
        msg.data = self.current_yaw
        self.yaw_pub.publish(msg)

    def trigger_callback(self, request, response):
        if not self.running:
            if not self.initialized or self.latest_measure_yaw is None:
                response.success = False
                response.message = "Waiting for measure to initialize yaw."
                return response

            # **Reset current_yaw to the latest measure value**
            self.current_yaw = self.latest_measure_yaw
            self.last_time = self.get_clock().now()
            self.running = True
            self.get_logger().info(f"Yaw integration started. Yaw reset to latest measure: {self.current_yaw:.3f} rad")
            response.success = True
            response.message = "Started"
        else:
            self.running = False
            self.get_logger().info("Yaw integration stopped.")
            response.success = True
            response.message = "Stopped"
        return response

    def quaternion_to_yaw(self, x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

def main(args=None):
    rclpy.init(args=args)
    node = YawIntegratorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
