import math
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32

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
    bearing = (bearing + 2 * math.pi) % (2 * math.pi)
    return bearing


class NavSatFollower(Node):
    def __init__(self):
        super().__init__('navsat_follower')

        # Declare parameters
        self.declare_parameter('points', [])
        self.declare_parameter('threshold', 2.0)  # meters

        # Get parameters
        points_list = self.get_parameter('points').get_parameter_value().double_array_value
        if len(points_list) % 2 != 0:
            self.get_logger().error('Parameter points must be a flat list of lat, lon pairs')
            raise ValueError('Invalid points parameter')

        self.points: List[Tuple[float, float]] = []
        for i in range(0, len(points_list), 2):
            self.points.append((points_list[i], points_list[i + 1]))

        self.threshold: float = self.get_parameter('threshold').get_parameter_value().double_value
        self.current_index = 0

        # Subscriber to GPS navsat_fix
        self.subscription = self.create_subscription(
            NavSatFix,
            'gps_sensor/navsat',
            self.gps_callback,
            10)

        # Publisher for yaw setpoint
        self.yaw_pub = self.create_publisher(Float32, 'yaw_set', 10)

        # Publisher for current GPS target at 1Hz
        self.target_pub = self.create_publisher(NavSatFix, 'gps_target', 10)
        self.target_timer = self.create_timer(1.0, self.publish_target)

        self.get_logger().info(f'Loaded {len(self.points)} target points, threshold={self.threshold}m')

    def gps_callback(self, msg: NavSatFix):
        if not self.points:
            self.get_logger().warn('No target points specified')
            return

        lat = msg.latitude
        lon = msg.longitude
        target_lat, target_lon = self.points[self.current_index]

        # Compute distance to current target
        dist = haversine_distance(lat, lon, target_lat, target_lon)
        self.get_logger().debug(f'Distance to point {self.current_index}: {dist:.2f} m')

        # Check if within threshold
        if dist <= self.threshold:
            self.get_logger().info(f'Reached point {self.current_index}, switching to next')
            self.current_index = (self.current_index + 1) % len(self.points)

        # Compute bearing (yaw) toward target
        # Use the updated current_index after potential switch
        target_lat, target_lon = self.points[self.current_index]
        yaw = bearing_to_target(lat, lon, target_lat, target_lon)
        self.get_logger().debug(f'Publishing yaw: {yaw:.3f} rad')

        # Publish yaw setpoint
        yaw_msg = Float32()
        yaw_msg.data = float(yaw)
        self.yaw_pub.publish(yaw_msg)

    def publish_target(self):
        """
        Periodically publish the current target waypoint as a NavSatFix message on 'gps_target'.
        """
        if not self.points:
            return

        target_lat, target_lon = self.points[self.current_index]
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gps_target'
        msg.latitude = target_lat
        msg.longitude = target_lon
        msg.altitude = 0.0
        # Optional: set covariance or other fields if needed
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
