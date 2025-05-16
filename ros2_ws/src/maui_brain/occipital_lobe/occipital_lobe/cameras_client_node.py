#!/usr/bin/env python3
import base64
import threading

import cv2
import numpy as np
import requests

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class StereoCameraClient(Node):
    def __init__(self):
        super().__init__('stereo_camera_client')

        # declare & read parameters
        self.declare_parameter('api_url', 'http://localhost:5000/capture')
        self.declare_parameter('frequency', 10.0)  # Hz
        self.api_url = self.get_parameter('api_url').value
        self.freq = float(self.get_parameter('frequency').value)

        # publishers
        self.left_pub = self.create_publisher(Image, '/left_camera', 10)
        self.right_pub = self.create_publisher(Image, '/right_camera', 10)

        # helper for conversions
        self.bridge = CvBridge()

        # timer to call API periodically
        period = 1.0 / self.freq
        self.timer = self.create_timer(period, self.capture_and_publish)

        # lock to protect concurrent calls (just in case)
        self.lock = threading.Lock()

        self.get_logger().info(
            f"StereoCameraClient started: calling {self.api_url} at {self.freq} Hz"
        )

        def capture_and_publish(self):
        with self.lock:
            try:
                resp = requests.get(self.api_url, timeout=1.0)
                resp.raise_for_status()
                data = resp.json()
            except Exception as e:
                self.get_logger().error(f"API call failed: {e}")
                return

            for key, pub in (('camera1', self.left_pub),
                             ('camera2', self.right_pub)):
                b64 = data.get(key, None)
                if b64 is None:
                    self.get_logger().error(f"No '{key}' in API response")
                    continue

                # decode JPEG
                jpg = base64.b64decode(b64)
                arr = np.frombuffer(jpg, np.uint8)
                img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                if img is None:
                    self.get_logger().error(f"Failed to decode {key}")
                    continue

                # convert BGR→RGB and publish as rgb8
                img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                msg = self.bridge.cv2_to_imgmsg(img_rgb, encoding='rgb8')
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = key
                pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = StereoCameraClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
