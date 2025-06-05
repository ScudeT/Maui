import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion, Pose
from nav_msgs.msg import Odometry

class OrientationToPosePublisher(Node):
    def __init__(self):
        super().__init__('orientation_to_pose_publisher')
        self.pose_ref_pub = self.create_publisher(Pose, 'pose_ref', 10)
        self.pose_est_pub = self.create_publisher(Pose, 'pose_est', 10)

        self.q_ref_sub = self.create_subscription(
            Quaternion, 'q_ref', self.q_ref_callback, 10)

        self.odom_est_sub = self.create_subscription(
            Odometry, 'odom_est', self.odom_est_callback, 10)

    def q_ref_callback(self, msg):
        pose_msg = Pose()
        pose_msg.orientation = msg
        # Position left at default (0,0,0)
        self.pose_ref_pub.publish(pose_msg)

    def odom_est_callback(self, msg):
        pose_msg = Pose()
        pose_msg.orientation = msg.pose.pose.orientation
        # Position left at default (0,0,0)
        self.pose_est_pub.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = OrientationToPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
