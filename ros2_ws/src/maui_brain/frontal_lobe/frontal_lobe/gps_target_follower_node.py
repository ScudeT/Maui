import math
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

# Earth radius in meters
EARTH_RADIUS = 6371000.0


def haversine_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """
    Compute the great-circle distance between two points on the Earth using the Haversine formula.
    """
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)

    a = math.sin(dphi / 2.0) ** 2 + \
        math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2.0) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    return EARTH_RADIUS * c


def bearing_to_target(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """
    Compute bearing from point 1 to point 2 relative to north (z axis up), returned in radians.
    0 = North, positive clockwise eastwards.
    """
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dlambda = math.radians(lon2 - lon1)

    y = math.sin(dlambda) * math.cos(phi2)
    x = math.cos(phi1) * math.sin(phi2) - \
        math.sin(phi1) * math.cos(phi2) * math.cos(dlambda)
    bearing = math.atan2(y, x)
    # Convert from radians (-pi,pi) to [0,2*pi)
    return (bearing + 2 * math.pi) % (2 * math.pi)


class NavSatFollower(Node):
    def __init__(self):
        super().__init__('navsat_follower')

        # Parameter descriptors
        points_desc = ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                                          description='Flat list of latitude, longitude pairs')
        thresh_desc = ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE,
                                          description='Distance threshold in meters')

        # Declare parameters with explicit types
        self.declare_parameter('points', [ 45.850235,9.396081, 45.859799,9.396632 ], points_desc)
        self.declare_parameter('threshold', 2.0, thresh_desc)

        # Retrieve parameters
        points_list = self.get_parameter('points').get_parameter_value().double_array_value
        if len(points_list) % 2 != 0:
            self.get_logger().error('Parameter points must be a flat list of lat, lon pairs')
            raise ValueError('Invalid points parameter')

        self.points: List[Tuple[float, float]] = []
        for i in range(0, len(points_list), 2):
            self.points.append((points_list[i], points_list[i + 1]))

        self.threshold: float = self.get_parameter('threshold').get_parameter_value().double_value
        self.current_index = 0

        # Subscribers and publishers
        self.subscription = self.create_subscription(
            NavSatFix,
            'gps_sensor/navsat',
            self.gps_callback,
            10)

        self.yaw_pub = self.create_publisher(Float32, 'yaw_set', 10)
        self.target_pub = self.create_publisher(NavSatFix, 'gps_target', 10)
        self.target_timer = self.create_timer(1.0, self.publish_target)

        self.get_logger().info(
            f'Loaded {len(self.points)} target points, threshold={self.threshold} m')

    def gps_callback(self, msg: NavSatFix):
        if not self.points:
            self.get_logger().warn('No target points specified')
            return

        lat, lon = msg.latitude, msg.longitude
        tgt_lat, tgt_lon = self.points[self.current_index]

        dist = haversine_distance(lat, lon, tgt_lat, tgt_lon)
        self.get_logger().debug(f'Distance to point {self.current_index}: {dist:.2f} m')

        if dist <= self.threshold:
            self.get_logger().info(f'Reached point {self.current_index}, switching to next')
            self.current_index = (self.current_index + 1) % len(self.points)

        # Update target after index change
        tgt_lat, tgt_lon = self.points[self.current_index]
        yaw = bearing_to_target(lat, lon, tgt_lat, tgt_lon)*180/math.pi  # Convert to degrees
        self.get_logger().debug(f'Publishing yaw: {yaw:.3f}°')

        yaw_msg = Float32()
        yaw_msg.data = yaw
        self.yaw_pub.publish(yaw_msg)

    def publish_target(self):
        """
        Publish the current target waypoint as NavSatFix on 'gps_target' at 1 Hz.
        """
        if not self.points:
            return

        tgt_lat, tgt_lon = self.points[self.current_index]
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gps_target'
        msg.latitude = tgt_lat
        msg.longitude = tgt_lon
        msg.altitude = 0.0
        self.target_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = NavSatFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down NavSatFollower')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
