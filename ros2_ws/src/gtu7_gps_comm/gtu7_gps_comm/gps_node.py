import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import gpsd

class GpsPublisher(Node):

    def __init__(self):
        super().__init__('gps_publisher')
        self.publisher_ = self.create_publisher(NavSatFix, 'gps_data', 10)

        # Frequency parameter
        self.declare_parameter('freq', 1.0)
        freq = self.get_parameter('freq').value
        period = 1.0 / freq if freq > 0 else 1.0

        # Connect to gpsd (adjust host/port if needed)
        gpsd.connect()

        self.timer = self.create_timer(period, self.publish_gps_data)

    def publish_gps_data(self):
        try:
            gps_data = gpsd.get_current()
        except UserWarning:
            self.get_logger().warn('GPS not active (no fix yet).')
            return

        if getattr(gps_data, 'mode', 0) < 2:
            self.get_logger().warn(f'GPS fix still not valid (mode={gps_data.mode}).')
            return

        msg = NavSatFix()
        msg.header.frame_id = "gps_link"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.latitude = gps_data.lat
        msg.longitude = gps_data.lon
        msg.altitude = gps_data.alt

        self.get_logger().info(
            f"Publishing GPS data: {msg.latitude:.6f}, {msg.longitude:.6f}, {msg.altitude:.2f}"
        )
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = GpsPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
