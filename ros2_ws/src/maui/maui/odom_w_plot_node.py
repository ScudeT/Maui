#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
from datetime import datetime

class OdomPlotter(Node):
    def __init__(self):
        super().__init__('odom_plotter')

        # Declare and get the image filename parameter
        self.declare_parameter('image_name', 'odom_plot.png')
        self.image_name = self.get_parameter('image_name').get_parameter_value().string_value

        # State: whether we are currently recording
        self.recording = False

        # Storage for timestamps and angular velocities
        self.times = []
        self.ang_x = []
        self.ang_y = []
        self.ang_z = []

        # Subscriber to /odom
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )

        # Service to toggle recording on/off
        self.snap_srv = self.create_service(
            Trigger,
            'snap',
            self.snap_callback
        )

        # Timer at 1 Hz to check for parameter updates
        self.param_timer = self.create_timer(1.0, self.timer_callback)

        self.get_logger().info('OdomPlotter node ready.')

    def odom_callback(self, msg: Odometry):
        if not self.recording:
            return
        # Record current ROS time (in seconds since start), or use wall time
        t = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9
        self.times.append(t)
        self.ang_x.append(msg.twist.twist.angular.x)
        self.ang_y.append(msg.twist.twist.angular.y)
        self.ang_z.append(msg.twist.twist.angular.z)
        self.get_logger().debug(f"Recorded angular vel @ {t:.3f}s: ({msg.twist.twist.angular.x:.3f}, "
                                f"{msg.twist.twist.angular.y:.3f}, {msg.twist.twist.angular.z:.3f})")

    def snap_callback(self, request, response):
        # Toggle recording state
        self.recording = not self.recording

        if self.recording:
            # Starting: clear any old data
            self.times.clear()
            self.ang_x.clear()
            self.ang_y.clear()
            self.ang_z.clear()
            response.success = True
            response.message = 'Recording started.'
            self.get_logger().info('Recording started.')
        else:
            # Stopping: plot & save
            self.plot_and_save()
            response.success = True
            response.message = f'Recording stopped. Plot saved to "{self.image_name}".'
            self.get_logger().info(f'Recording stopped. Plot saved to "{self.image_name}".')
        return response

    def timer_callback(self):
        # Check for updates to the image_name parameter
        param = self.get_parameter('image_name').get_parameter_value().string_value
        if param != self.image_name:
            self.image_name = param
            self.get_logger().info(f'Image filename parameter changed: {self.image_name}')

    def plot_and_save(self):
        if not self.times:
            self.get_logger().warn('No data recorded; nothing to plot.')
            return

        plt.figure()
        plt.plot(self.times, self.ang_x, label='angular.x')
        plt.plot(self.times, self.ang_y, label='angular.y')
        plt.plot(self.times, self.ang_z, label='angular.z')
        plt.xlabel('Time [s]')
        plt.ylabel('Angular velocity [rad/s]')
        plt.title('Odom Angular Velocities')
        plt.legend()
        plt.tight_layout()
        plt.savefig(self.image_name)
        plt.close()

def main(args=None):
    rclpy.init(args=args)
    node = OdomPlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
