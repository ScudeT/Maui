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

        # --- parameters ---
        self.declare_parameter('uere', 5.0)
        self.uere = self.get_parameter('uere').value

        # --- NMEA‐derived quality metrics ---
        self.hdop = None
        self.vdop = None
        self.sigma_lat = None
        self.sigma_lon = None
        self.sigma_alt = None

        # --- publisher ---
        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        self.pub = self.create_publisher(NavSatFix, 'gps_data', qos)

        # --- open serial port ---
        try:
            self.ser = serial.Serial('/dev/ttyAMA0', baudrate=9600, timeout=1.0)
            self.get_logger().info('Opened /dev/ttyAMA0 @9600')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            raise

        # --- timer to poll GPS ---
        self.create_timer(0.1, self.timer_callback)

        self.get_logger().info('GPS node initialized; entering spin-loop')

    def timer_callback(self):
        # read raw line
        try:
            raw = self.ser.readline()
            line = raw.decode('ascii', errors='replace').strip()
        except Exception as e:
            self.get_logger().warning(f'Serial read error: {e}')
            return

        if not line:
            self.get_logger().debug('No data read from serial')
            return

        self.get_logger().debug(f'NMEA raw: {line}')

        if not line.startswith('$'):
            self.get_logger().debug('Line does not start with $, skipping')
            return

        # try parse
        try:
            msg = pynmea2.parse(line)
        except pynmea2.ParseError as e:
            self.get_logger().debug(f'Parse error: {e}')
            return

        stype = msg.sentence_type
        self.get_logger().debug(f'Parsed sentence type: {stype}')

        # GSA → update DOP
        if stype == 'GSA':
            hd = getattr(msg, 'hdop', None)
            vd = getattr(msg, 'vdop', None)
            if hd not in (None, '') and vd not in (None, ''):
                try:
                    self.hdop = float(hd)
                    self.vdop = float(vd)
                    self.get_logger().info(f'GSA: HDOP={self.hdop}, VDOP={self.vdop}')
                except ValueError:
                    self.get_logger().warning('GSA values not numeric')
            return

        # GST → update sigma
        if stype == 'GST':
            lat_sd = getattr(msg, 'std_dev_latitude', None)
            lon_sd = getattr(msg, 'std_dev_longitude', None)
            alt_sd = getattr(msg, 'std_dev_altitude', None)
            if None not in (lat_sd, lon_sd, alt_sd) and '' not in (lat_sd, lon_sd, alt_sd):
                try:
                    self.sigma_lat = float(lat_sd)
                    self.sigma_lon = float(lon_sd)
                    self.sigma_alt = float(alt_sd)
                    self.get_logger().info(
                        f'GST: σ_lat={self.sigma_lat}, σ_lon={self.sigma_lon}, σ_alt={self.sigma_alt}'
                    )
                except ValueError:
                    self.get_logger().warning('GST values not numeric')
            return

        # Only RMC or GGA fixes
        if stype == 'RMC':
            status = getattr(msg, 'status', None)
            self.get_logger().debug(f'RMC status: {status}')
            if status != 'A':
                self.get_logger().debug('RMC void fix, skipping')
                return
        elif stype == 'GGA':
            qual = getattr(msg, 'gps_qual', None)
            self.get_logger().debug(f'GGA gps_qual: {qual}')
            if qual in (None, '0', 0):
                self.get_logger().debug('GGA no fix, skipping')
                return
        else:
            self.get_logger().debug('Not RMC or GGA, skipping')
            return

        # parse lat/lon
        lat_s = getattr(msg, 'latitude', None)
        lon_s = getattr(msg, 'longitude', None)
        if lat_s in (None, '') or lon_s in (None, ''):
            self.get_logger().debug('Latitude or longitude empty, skipping')
            return
        try:
            lat = float(lat_s)
            lon = float(lon_s)
        except ValueError:
            self.get_logger().warning('Latitude/Longitude conversion failed')
            return

        # parse altitude
        if stype == 'GGA':
            alt_s = getattr(msg, 'altitude', None)
            if alt_s not in (None, ''):
                try:
                    alt = float(alt_s)
                except ValueError:
                    alt = math.nan
            else:
                alt = math.nan
        else:
            alt = math.nan

        # build NavSatFix
        nav = NavSatFix()
        nav.header.stamp = self.get_clock().now().to_msg()
        nav.header.frame_id = 'gps'
        nav.latitude = lat
        nav.longitude = lon
        nav.altitude = alt

        # status field
        if stype == 'GGA':
            nav.status.status = (
                NavSatStatus.STATUS_FIX if int(msg.gps_qual) > 0 else NavSatStatus.STATUS_NO_FIX
            )
        else:
            nav.status.status = NavSatStatus.STATUS_FIX
        nav.status.service = NavSatStatus.SERVICE_GPS

        # compute covariance
        cov = None
        cov_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

        if None not in (self.sigma_lat, self.sigma_lon, self.sigma_alt):
            Cxx = self.sigma_lon**2
            Cyy = self.sigma_lat**2
            Czz = self.sigma_alt**2
            cov = [Cxx,0,0, 0,Cyy,0, 0,0,Czz]
            cov_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

        elif None not in (self.hdop, self.vdop):
            C_h = (self.hdop * self.uere)**2
            C_v = (self.vdop * self.uere)**2
            cov = [C_h,0,0, 0,C_h,0, 0,0,C_v]
            cov_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

        if cov is not None:
            nav.position_covariance = cov
            nav.position_covariance_type = cov_type
        else:
            nav.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

        # publish
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
        node.get_logger().info('Shutting down')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
