import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import requests
import cv2
import numpy as np

class ImageClientNode(Node):
    def __init__(self):
        super().__init__('image_client_node')
        # Publisher for sensor_msgs/Image
        self.publisher_ = self.create_publisher(Image, 'camera/image', 10)
        # Bridge to convert between OpenCV and ROS Image
        self.bridge = CvBridge()
        # Timer at 10 Hz
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Image client node started, polling server at 10Hz')

    def timer_callback(self):
        try:
            # Fetch JPEG image from server
            response = requests.get('http://0.0.0.0:5000/')
            response.raise_for_status()
            # Decode JPEG to OpenCV BGR image
            np_arr = np.frombuffer(response.content, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            # Convert BGR OpenCV image to ROS Image message
            ros_image = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            ros_image.header.stamp = self.get_clock().now().to_msg()
            ros_image.header.frame_id = 'camera'

            # Publish ROS Image
            self.publisher_.publish(ros_image)
            self.get_logger().debug('Published image frame')

        except Exception as e:
            self.get_logger().error(f'Failed to fetch or publish image: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ImageClientNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
