import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import gpsd

class GpsPublisher(Node):

    def __init__(self):
        super().__init__('gps_publisher')
        self.publisher_ = self.create_publisher(NavSatFix, 'gps_data', 10)

        # Declare a parameter for frequency (Hz) with a default value of 2.0 Hz
        self.declare_parameter('freq', 1.0)
        freq = self.get_parameter('freq').value

        # Compute the timer period as 1 / frequency
        period = 1.0 / freq if freq > 0 else 0.5

        # Initialize gpsd session
        gpsd.connect()

        # Create a timer with the computed period
        self.timer = self.create_timer(period, self.publish_gps_data)

    def publish_gps_data(self):
        # Get GPS data
        gps_data = gpsd.get_current()

        # Check if the GPS data has a valid fix
        msg = NavSatFix()

        msg.header.frame_id = "gps_link"
        msg.header.stamp = self.get_clock().now().to_msg()
            
        msg.latitude = gps_data.lat
        msg.longitude = gps_data.lon
        msg.altitude = gps_data.alt

            # Log and publish the message
        self.get_logger().info(f"Publishing GPS data: {msg.latitude}, {msg.longitude}, {msg.altitude}")
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    gps_publisher = GpsPublisher()
    
    try:
        rclpy.spin(gps_publisher)
    except KeyboardInterrupt:
        pass

    gps_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
