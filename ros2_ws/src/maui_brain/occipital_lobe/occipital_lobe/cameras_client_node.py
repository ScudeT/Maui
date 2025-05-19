import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import requests
import cv2
import numpy as np

class DualCameraClientNode(Node):
    def __init__(self):
        super().__init__('dual_camera_client')

        # Declare and get parameter for capture frequency (Hz)
        self.declare_parameter('capture_frequency', 10.0)
        freq = self.get_parameter('capture_frequency').value
        self.get_logger().info(f"Capture frequency set to {freq} Hz")

        # Publishers for each camera
        self.pub_cam1 = self.create_publisher(Image, '/camera/1', 10)
        self.pub_cam2 = self.create_publisher(Image, '/camera/2', 10)

        self.bridge = CvBridge()

        # Timer period based on frequency
        timer_period = 1.0 / freq
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # URLs of the capture service
        self.url_cam1 = 'http://0.0.0.0:5000/capture/cam1'
        self.url_cam2 = 'http://0.0.0.0:5000/capture/cam2'

    def timer_callback(self):
        try:
            # Fetch raw JPEG bytes for camera 1
            resp1 = requests.get(self.url_cam1, timeout=1.0)
            resp1.raise_for_status()
            img_bytes1 = resp1.content

            # Decode JPEG bytes to OpenCV image (BGR)
            np_arr1 = np.frombuffer(img_bytes1, np.uint8)
            cv_img1 = cv2.imdecode(np_arr1, cv2.IMREAD_COLOR)
            if cv_img1 is None:
                self.get_logger().error("Failed to decode image from camera 1")
                return

            # Fetch raw JPEG bytes for camera 2
            resp2 = requests.get(self.url_cam2, timeout=1.0)
            resp2.raise_for_status()
            img_bytes2 = resp2.content

            np_arr2 = np.frombuffer(img_bytes2, np.uint8)
            cv_img2 = cv2.imdecode(np_arr2, cv2.IMREAD_COLOR)
            if cv_img2 is None:
                self.get_logger().error("Failed to decode image from camera 2")
                return

            # Convert to ROS Image messages
            ros_img1 = self.bridge.cv2_to_imgmsg(cv_img1, encoding='bgr8')
            ros_img1.header.stamp = self.get_clock().now().to_msg()
            ros_img1.header.frame_id = 'camera_1'

            ros_img2 = self.bridge.cv2_to_imgmsg(cv_img2, encoding='bgr8')
            ros_img2.header.stamp = self.get_clock().now().to_msg()
            ros_img2.header.frame_id = 'camera_2'

            # Publish images
            self.pub_cam1.publish(ros_img1)
            self.pub_cam2.publish(ros_img2)

            self.get_logger().debug("Published images from both cameras")

        except Exception as e:
            self.get_logger().error(f"Exception during image fetch or publish: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = DualCameraClientNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
