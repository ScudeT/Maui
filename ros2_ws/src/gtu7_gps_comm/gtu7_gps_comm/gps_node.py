import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import gpsd
import socket

class GpsPublisher(Node):

    def __init__(self):
        super().__init__('gps_publisher')
        self.publisher_ = self.create_publisher(NavSatFix, 'gps_data', 10)

        self.declare_parameter('freq', 1.0)
        freq = self.get_parameter('freq').value
        period = 1.0 / freq if freq > 0 else 1.0

        # Use the correct host for GPSD (adjust as needed)
        gpsd_host = self.declare_parameter('gpsd_host', 'localhost').value
        gpsd_port = self.declare_parameter('gpsd_port', 2947).value

        try:
            gpsd.connect(host=gpsd_host, port=gpsd_port)
            self.get_logger().info(f"Connected to gpsd at {gpsd_host}:{gpsd_port}")
        except (socket.error, Exception) as e:
            self.get_logger().error(f"Could not connect to gpsd: {e}")
            raise SystemExit(1)

        self.last_fix_mode = None
        self.timer = self.create_timer(period, self.publish_gps_data)

    def publish_gps_data(self):
        try:
            gps_data = gpsd.get_current()
        except Exception as e:
            self.get_logger().warn(f'Failed to read GPS data: {e}')
            return

        # Mode: 1 = no fix, 2 = 2D fix, 3 = 3D fix
        mode = getattr(gps_data, 'mode', 0)
        if mode < 2:
            if self.last_fix_mode != mode:
                self.get_logger().warn(f'GPS fix not valid (mode={mode}). Waiting for fix...')
            self.last_fix_mode = mode
            return

        try:
            msg = NavSatFix()
            msg.header.frame_id = "gps_link"
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.latitude = float(gps_data.lat)
            msg.longitude = float(gps_data.lon)
            msg.altitude = float(gps_data.alt)

            self.get_logger().info(
                f"Publishing GPS data: {msg.latitude:.6f}, {msg.longitude:.6f}, {msg.altitude:.2f}"
            )
            self.publisher_.publish(msg)
        except AttributeError as e:
            self.get_logger().warn(f"GPS data missing expected fields: {e}")

    def dmm_to_dd(val):
        degrees = int(val // 100)
        minutes = val - degrees * 100
        return degrees + minutes / 60

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
