import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
import numpy as np
from scipy.spatial.transform import Rotation as R

class SinusoidalQuaternionPublisher(Node):
    def __init__(self):
        super().__init__('sinusoidal_quaternion_publisher')
        self.declare_parameter('frequency', 10.0)
        self.declare_parameter('amplitude', np.pi/2)
        self.declare_parameter('period', 10.0)
        self.declare_parameter('phase_shift', np.pi/2)
        self.declare_parameter('yaw_min', -np.pi/2)
        self.declare_parameter('yaw_max', np.pi/2)
        
        self.freq = self.get_parameter('frequency').value
        self.amp = self.get_parameter('amplitude').value
        self.T = self.get_parameter('period').value
        self.phase_shift = self.get_parameter('phase_shift').value
        self.yaw_min = self.get_parameter('yaw_min').value
        self.yaw_max = self.get_parameter('yaw_max').value

        self.pub_mes_odom = self.create_publisher(Odometry, 'odom', 10)
        self.pub_ref = self.create_publisher(Quaternion, 'q_ref', 10)

        self.t0 = self.get_clock().now()
        self.prev_time = None
        self.prev_yaw_mes = None

        self.timer = self.create_timer(1.0/self.freq, self.timer_callback)

    def timer_callback(self):
        now = self.get_clock().now()
        t = (now - self.t0).nanoseconds * 1e-9

        # Yaw signals (sinusoids)
        yaw_mes = self.amp * np.sin(2 * np.pi * t / self.T)
        yaw_ref = self.amp * np.sin(2 * np.pi * t / self.T + self.phase_shift)

        # Clamp yaws
        yaw_mes = np.clip(yaw_mes, self.yaw_min, self.yaw_max)
        yaw_ref = np.clip(yaw_ref, self.yaw_min, self.yaw_max)

        # Derivative of yaw_mes (numerical)
        wz = 0.0
        if self.prev_time is not None and self.prev_yaw_mes is not None:
            dt = t - self.prev_time
            if dt > 0:
                dyaw = yaw_mes - self.prev_yaw_mes
                # Unwrap for continuity
                dyaw = np.arctan2(np.sin(dyaw), np.cos(dyaw))
                wz = dyaw / dt
        self.prev_time = t
        self.prev_yaw_mes = yaw_mes

        # Euler to quaternion (roll=0, pitch=0, yaw)
        q_mes_np = R.from_euler('zyx', [yaw_mes, 0, 0]).as_quat()
        q_ref_np = R.from_euler('zyx', [yaw_ref, 0, 0]).as_quat()

        q_mes = Quaternion()
        q_mes.x, q_mes.y, q_mes.z, q_mes.w = q_mes_np

        q_ref = Quaternion()
        q_ref.x, q_ref.y, q_ref.z, q_ref.w = q_ref_np

        # Publish Odometry with q_mes and wz
        odom_msg = Odometry()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.orientation = q_mes
        odom_msg.twist.twist.angular.z = float(wz)
        self.pub_mes_odom.publish(odom_msg)

        # Publish q_ref as before
        self.pub_ref.publish(q_ref)

def main(args=None):
    rclpy.init(args=args)
    node = SinusoidalQuaternionPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
