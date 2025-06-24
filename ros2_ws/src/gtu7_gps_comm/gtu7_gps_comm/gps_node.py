#!/usr/bin/env python3
import math
import serial
import pynmea2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import NavSatFix, NavSatStatus

class Gtu7GpsNode(Node):
    def __init__(self):
        super().__init__('gtu7_gps_node')

        # Parameters
        self.declare_parameter('uere', 5.0)  # User equivalent range error (meters)
        self.uere = self.get_parameter('uere').value

        # NMEA-derived quality metrics
        self.hdop = None
        self.vdop = None
        self.sigma_lat = None
        self.sigma_lon = None
        self.sigma_alt = None

        # Publisher with best-effort, volatile QoS
        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        self.pub = self.create_publisher(NavSatFix, 'gps_data', qos)

        # Open serial port
        try:
            self.ser = serial.Serial('/dev/ttyAMA0', baudrate=9600, timeout=1.0)
            self.get_logger().info('Opened serial port /dev/ttyAMA0 @9600')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open /dev/ttyAMA0: {e}')
            raise

        # Timer to poll GPS
        self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        """Read NMEA line, parse, update metrics or publish NavSatFix."""
        try:
            raw = self.ser.readline()
            line = raw.decode('ascii', errors='replace').strip()
        except Exception as e:
            self.get_logger().warning(f'Serial read error: {e}')
            return

        if not line.startswith('$'):
            return

        try:
            msg = pynmea2.parse(line)
        except pynmea2.ParseError:
            return

        stype = msg.sentence_type  # e.g. "GGA", "RMC", "GSA", "GST", etc.

        # Update DOP from GSA
        if stype == 'GSA':
            try:
                self.hdop = float(msg.hdop)
                self.vdop = float(msg.vdop)
                self.get_logger().debug(f'Updated HDOP={self.hdop}, VDOP={self.vdop}')
            except (AttributeError, ValueError):
                pass
            return

        # Update sigma estimates from GST
        if stype == 'GST':
            try:
                self.sigma_lat = float(msg.std_dev_latitude)
                self.sigma_lon = float(msg.std_dev_longitude)
                self.sigma_alt = float(msg.std_dev_altitude)
                self.get_logger().debug(
                    f'Updated σ_lat={self.sigma_lat}, σ_lon={self.sigma_lon}, σ_alt={self.sigma_alt}'
                )
            except (AttributeError, ValueError):
                pass
            return

        # Only process fixes from GGA or RMC
        if stype == 'RMC':
            # RMC status: 'A' = valid, 'V' = void
            if getattr(msg, 'status', None) != 'A':
                return
        elif stype == 'GGA':
            # GGA gps_qual: '0' = invalid
            if getattr(msg, 'gps_qual', '0') == '0':
                return
        else:
            return

        # Parse latitude/longitude
        try:
            lat = float(msg.latitude)
            lon = float(msg.longitude)
        except (TypeError, ValueError):
            return

        # Parse altitude if GGA, else NaN
        if stype == 'GGA':
            try:
                alt = float(msg.altitude)
            except (AttributeError, ValueError):
                alt = math.nan
        else:
            alt = math.nan

        # Build NavSatFix message
        nav = NavSatFix()
        nav.header.stamp = self.get_clock().now().to_msg()
        nav.header.frame_id = 'gps'

        # Fill status field
        if stype == 'GGA':
            nav.status.status = (
                NavSatStatus.STATUS_FIX
                if int(msg.gps_qual) > 0
                else NavSatStatus.STATUS_NO_FIX
            )
        else:
            nav.status.status = NavSatStatus.STATUS_FIX
        nav.status.service = NavSatStatus.SERVICE_GPS

        nav.latitude = lat
        nav.longitude = lon
        nav.altitude = alt

        # Compute covariance
        cov = None
        cov_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

        if self.sigma_lat is not None and self.sigma_lon is not None and self.sigma_alt is not None:
            # Use GST sigma estimates directly
            Cxx = self.sigma_lon ** 2
            Cyy = self.sigma_lat ** 2
            Czz = self.sigma_alt ** 2
            cov = [Cxx, 0.0, 0.0,
                   0.0, Cyy, 0.0,
                   0.0, 0.0, Czz]
            cov_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

        elif self.hdop is not None and self.vdop is not None:
            # Approximate using DOP metrics and UERE
            C_horiz = (self.hdop * self.uere) ** 2
            C_vert  = (self.vdop  * self.uere) ** 2
            cov = [C_horiz, 0.0,     0.0,
                   0.0,     C_horiz, 0.0,
                   0.0,     0.0,     C_vert]
            cov_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

        if cov is not None:
            nav.position_covariance = cov
            nav.position_covariance_type = cov_type
        else:
            nav.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

        # Publish and log
        self.pub.publish(nav)
        self.get_logger().info(
            f'Published [{stype}] fix: lat={lat:.6f}, lon={lon:.6f}, '
            f'alt={alt if not math.isnan(alt) else "n/a"}, cov_type={cov_type}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = Gtu7GpsNode()
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
