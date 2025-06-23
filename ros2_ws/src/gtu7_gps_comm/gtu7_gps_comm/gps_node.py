#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
import serial
import pynmea2

class GpsNode(Node):
    def __init__(self):
        super().__init__('gtu7_gps_node')
        # Publisher for NavSatFix messages
        self.pub = self.create_publisher(NavSatFix, 'gps_data', 10)

        # Open the serial port
        try:
            self.ser = serial.Serial(
                port='/dev/ttyAMA0',
                baudrate=9600,
                timeout=1.0
            )
            self.get_logger().info('Opened serial port /dev/ttyAMA0')
        except serial.SerialException as e:
            self.get_logger().error(f'Could not open serial port: {e}')
            raise

        # Timer to poll the serial port
        self.timer = self.create_timer(0.1, self.read_serial)

    def read_serial(self):
        """Read from serial, parse NMEA, and publish NavSatFix if position is valid."""
        try:
            line = self.ser.readline().decode('ascii', errors='replace').strip()
        except Exception as e:
            self.get_logger().warn(f'Serial read error: {e}')
            return

        if not line.startswith('$'):
            return

        try:
            msg = pynmea2.parse(line)
        except pynmea2.ParseError:
            return

        # Only handle GGA (fix data) or RMC (recommended minimum) sentences
        if isinstance(msg, (pynmea2.types.talker.GGA, pynmea2.types.talker.RMC)):
            lat = msg.latitude
            lon = msg.longitude
            # Skip if no fix / no lat-lon
            if lat is None or lon is None:
                return

            nav = NavSatFix()
            nav.header.stamp = self.get_clock().now().to_msg()
            nav.header.frame_id = 'gps'
            # Status: FIX (0=no_fix, 1=fix, 2=differential)
            nav.status.status = NavSatStatus.STATUS_FIX if getattr(msg, 'gps_qual', True) else NavSatStatus.STATUS_NO_FIX
            nav.status.service = NavSatStatus.SERVICE_GPS

            nav.latitude = lat
            nav.longitude = lon
            # Altitude in GGA; RMC has no altitude
            nav.altitude = getattr(msg, 'altitude', float('nan'))

            # We don't know covariance: set to unknown
            nav.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

            self.pub.publish(nav)
            self.get_logger().debug(f'Published NavSatFix: lat={lat}, lon={lon}, alt={nav.altitude}')

def main(args=None):
    rclpy.init(args=args)
    node = GpsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down GPS node')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()