import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
import numpy as np
from scipy.spatial.transform import Rotation as R
from tf2_ros import TransformBroadcaster

class SineOdomPublisher(Node):
    def __init__(self):
        super().__init__('figure8_odom_publisher')

        self.declare_parameter('frequency', 20.0)
        self.declare_parameter('A', 0.5)  # Roll amplitude (rad)
        self.declare_parameter('B', 0.5)  # Pitch amplitude (rad)
        self.declare_parameter('C', 1.0)  # Yaw amplitude (rad)
        self.declare_parameter('D', 1.0)  # Depth amplitude (m)
        self.declare_parameter('period', 10.0)  # seconds for one cycle

        self.freq = self.get_parameter('frequency').value
        self.A = self.get_parameter('A').value
        self.B = self.get_parameter('B').value
        self.C = self.get_parameter('C').value
        self.D = self.get_parameter('D').value
        self.period = self.get_parameter('period').value

        self.odom_pub = self.create_publisher(Odometry, '/est/odom', 10)
        self.t0 = self.get_clock().now()
        self.timer = self.create_timer(1.0/self.freq, self.timer_callback)

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

    def timer_callback(self):
        t = (self.get_clock().now() - self.t0).nanoseconds * 1e-9
        w = 2 * np.pi / self.period

        # Figure-8 parametric orientation (in radians)
        roll  = self.A * np.sin(w * t)
        pitch = self.B * np.sin(w * t)
        yaw   = self.C * np.sin(w * t) 

        quat = R.from_euler('xyz', [roll, pitch, yaw]).as_quat()  # [x, y, z, w]

        # Position figure-8 for fun (optional)
        x = 0.0
        y = 0.0
        z = self.D * np.sin(w * t)

        # Publish Odometry
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = z
        msg.pose.pose.orientation.x = quat[0]
        msg.pose.pose.orientation.y = quat[1]
        msg.pose.pose.orientation.z = quat[2]
        msg.pose.pose.orientation.w = quat[3]
        self.odom_pub.publish(msg)

        # Publish TF
        tf_msg = TransformStamped()
        tf_msg.header.stamp = msg.header.stamp
        tf_msg.header.frame_id = "odom"
        tf_msg.child_frame_id = "base_link"
        tf_msg.transform.translation.x = x
        tf_msg.transform.translation.y = y
        tf_msg.transform.translation.z = z
        tf_msg.transform.rotation.x = quat[0]
        tf_msg.transform.rotation.y = quat[1]
        tf_msg.transform.rotation.z = quat[2]
        tf_msg.transform.rotation.w = quat[3]
        self.tf_broadcaster.sendTransform(tf_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SineOdomPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
