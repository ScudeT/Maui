#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
import serial
import pynmea2
import math

class Gtu7GpsNode(Node):
    def __init__(self):
        super().__init__('gtu7_gps_node')
        self.pub = self.create_publisher(NavSatFix, 'gps_data', 10)

        # open serial port
        try:
            self.ser = serial.Serial('/dev/ttyAMA0', baudrate=9600, timeout=1.0)
            self.get_logger().info('Opened /dev/ttyAMA0')
        except serial.SerialException as e:
            self.get_logger().error(f"Serial error: {e}")
            raise

        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        line = None
        try:
            raw = self.ser.readline()
            line = raw.decode('ascii', errors='replace').strip()
        except Exception as e:
            self.get_logger().warn(f"Read error: {e}")
            return

        if not line.startswith('$'):
            return

        try:
            msg = pynmea2.parse(line)
        except pynmea2.ParseError:
            return

        # only GGA and RMC have position
        if not isinstance(msg, (pynmea2.types.talker.GGA,
                                pynmea2.types.talker.RMC)):
            return

        # convert lat/lon to floats, skip if invalid
        try:
            lat = float(msg.latitude)
            lon = float(msg.longitude)
        except (TypeError, ValueError):
            return

        # if no valid fix yet, latitude/longitude might be zero
        # but we'll still publish 0,0 if that's actually what the receiver says
        # retrieve altitude if present (GGA), else NaN
        if hasattr(msg, 'altitude'):
            try:
                alt = float(msg.altitude)
            except (TypeError, ValueError):
                alt = math.nan
        else:
            alt = math.nan

        # build NavSatFix
        nav = NavSatFix()
        nav.header.stamp = self.get_clock().now().to_msg()
        nav.header.frame_id = 'gps'

        # status: for GGA use gps_qual (0=no fix), else RMC we assume a fix if lat/lon parsed
        if hasattr(msg, 'gps_qual'):
            nav.status.status = (
                NavSatStatus.STATUS_FIX
                if int(msg.gps_qual) > 0 else NavSatStatus.STATUS_NO_FIX
            )
        else:
            nav.status.status = NavSatStatus.STATUS_FIX

        nav.status.service = NavSatStatus.SERVICE_GPS

        nav.latitude = lat
        nav.longitude = lon
        nav.altitude = alt

        # unknown covariance
        nav.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

        self.pub.publish(nav)
        self.get_logger().debug(f"Published fix: {lat}, {lon}, alt={alt}")

def main(args=None):
    rclpy.init(args=args)
    node = Gtu7GpsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
