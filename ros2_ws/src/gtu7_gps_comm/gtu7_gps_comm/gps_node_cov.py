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
        self.declare_parameter('uere', 5.0)  # meters
        self.uere = self.get_parameter('uere').value

        # NMEA‐derived quality metrics
        self.hdop = None
        self.vdop = None
        self.sigma_lat = None
        self.sigma_lon = None
        self.sigma_alt = None

        # Publisher
        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        self.pub = self.create_publisher(NavSatFix, 'gps_data', qos)

        # Open serial port
        try:
            self.ser = serial.Serial('/dev/ttyAMA0', baudrate=9600, timeout=1.0)
            self.get_logger().info('Opened /dev/ttyAMA0 @9600')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            raise

        self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
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

        stype = msg.sentence_type

        # —— GSA: update HDOP/VDOP —— 
        if stype == 'GSA':
            if getattr(msg, 'hdop', None) not in (None, '') and getattr(msg, 'vdop', None) not in (None, ''):
                try:
                    self.hdop = float(msg.hdop)
                    self.vdop = float(msg.vdop)
                    self.get_logger().debug(f'HDOP={self.hdop}, VDOP={self.vdop}')
                except ValueError:
                    pass
            return

        # —— GST: update σ estimates —— 
        if stype == 'GST':
            lat_sd = getattr(msg, 'std_dev_latitude', None)
            lon_sd = getattr(msg, 'std_dev_longitude', None)
            alt_sd = getattr(msg, 'std_dev_altitude', None)
            if lat_sd not in (None, '') and lon_sd not in (None, '') and alt_sd not in (None, ''):
                try:
                    self.sigma_lat = float(lat_sd)
                    self.sigma_lon = float(lon_sd)
                    self.sigma_alt = float(alt_sd)
                    self.get_logger().debug(
                        f'σ_lat={self.sigma_lat}, σ_lon={self.sigma_lon}, σ_alt={self.sigma_alt}'
                    )
                except ValueError:
                    pass
            return

        # —— Only process valid RMC/GGA fixes —— 
        if stype == 'RMC':
            if getattr(msg, 'status', None) != 'A':
                return
        elif stype == 'GGA':
            if getattr(msg, 'gps_qual', None) in (None, '0', 0):
                return
        else:
            return

        # —— Parse lat/lon —— 
        if getattr(msg, 'latitude', None) in (None, '') or getattr(msg, 'longitude', None) in (None, ''):
            return
        try:
            lat = float(msg.latitude)
            lon = float(msg.longitude)
        except ValueError:
            return

        # —— Parse altitude if GGA —— 
        if stype == 'GGA' and getattr(msg, 'altitude', None) not in (None, ''):
            try:
                alt = float(msg.altitude)
            except ValueError:
                alt = math.nan
        else:
            alt = math.nan

        # —— Build NavSatFix —— 
        nav = NavSatFix()
        nav.header.stamp = self.get_clock().now().to_msg()
        nav.header.frame_id = 'gps'

        # status
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

        # —— Compute covariance —— 
        cov = None
        cov_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

        # 1) GST-based
        if self.sigma_lat is not None and self.sigma_lon is not None and self.sigma_alt is not None:
            Cxx = self.sigma_lon ** 2
            Cyy = self.sigma_lat ** 2
            Czz = self.sigma_alt ** 2
            cov = [Cxx,0,0, 0,Cyy,0, 0,0,Czz]
            cov_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

        # 2) DOP-based
        elif self.hdop is not None and self.vdop is not None:
            C_h = (self.hdop * self.uere) ** 2
            C_v = (self.vdop * self.uere) ** 2
            cov = [C_h,0,0, 0,C_h,0, 0,0,C_v]
            cov_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

        if cov is not None:
            nav.position_covariance = cov
            nav.position_covariance_type = cov_type
        else:
            nav.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

        # —— Publish —— 
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
