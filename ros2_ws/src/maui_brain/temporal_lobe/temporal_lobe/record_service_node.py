import os
import time
import importlib

import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
import rosbag2_py
from std_srvs.srv import Trigger


class DynamicBagRecorder(Node):
    def __init__(self):
        super().__init__('dynamic_bag_recorder')
        # Declare parameters with default values
        self.declare_parameter('topics', ['/imu/data', '/gps_data', '/ms5837/pose'])
        self.declare_parameter('topic_types', ['sensor_msgs/msg/Imu', 'sensor_msgs/msg/NavSatFix', 'geometry_msgs/msg/PoseWithCovarianceStamped'])
        self.declare_parameter('parent_folder', 'bags')

        # Retrieve parameter values
        self.topics = self.get_parameter('topics').value
        self.topic_types = self.get_parameter('topic_types').value
        self.parent_folder = self.get_parameter('parent_folder').value

        # Check if topics and topic_types lists have the same length
        if len(self.topics) != len(self.topic_types):
            self.get_logger().error("The number of topics and topic_types must be equal!")
            rclpy.shutdown()
            return

        # Create subscriptions for each topic
        self.subscriptions_ = []
        for topic, type_str in zip(self.topics, self.topic_types):
            msg_type = self.get_msg_type_from_str(type_str)
            # Use a lambda with a default argument to capture the topic name
            subscription = self.create_subscription(
                msg_type,
                topic,
                lambda msg, t=topic: self.topic_callback(msg, t),
                10
            )
            self.subscriptions_.append(subscription)

        # Recording is off by default; writer will be created when recording starts.
        self.recording = False
        self.writer = None

        # Create the Trigger service to toggle recording
        self.srv = self.create_service(Trigger, '/record', self.toggle_recording_callback)
        self.get_logger().info("DynamicBagRecorder started with recording OFF.")

    def get_msg_type_from_str(self, type_str: str):
        """
        Given a type string like "std_msgs/msg/String", dynamically import and return the message class.
        """
        package, subfolder, msg = type_str.split('/')
        module_name = package + ".msg"
        mod = importlib.import_module(module_name)
        return getattr(mod, msg)

    def start_recording(self):
        """
        Starts recording by creating a new bag writer instance that saves data in a subfolder under parent_folder.
        """
        if self.writer is not None:
            self.get_logger().warn("Recording is already in progress.")
            return

        # Create a unique folder for this recording session
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        bag_folder = os.path.join(self.parent_folder, f"bag_{timestamp}")
        # os.makedirs(bag_folder, exist_ok=True)

        self.get_logger().info(f"Starting recording in folder: {bag_folder}")
        self.writer = rosbag2_py.SequentialWriter()
        storage_options = rosbag2_py.StorageOptions(
            uri=bag_folder,
            storage_id='sqlite3'
        )
        converter_options = rosbag2_py.ConverterOptions('', '')
        self.writer.open(storage_options, converter_options)

        # Create topics for the writer
        for topic, type_str in zip(self.topics, self.topic_types):
            topic_info = rosbag2_py.TopicMetadata(
                id=0,  # The id field is not critical in this simple example
                name=topic,
                type=type_str,
                serialization_format='cdr'
            )
            self.writer.create_topic(topic_info)

    def stop_recording(self):
        """
        Stops the current recording session.
        """
        if self.writer is None:
            self.get_logger().warn("No active recording to stop.")
            return

        self.get_logger().info("Stopping recording.")
        # In a complete implementation you might need to properly close the writer.
        # For this simple example, we simply set it to None.
        self.writer = None

    def toggle_recording_callback(self, request, response):
        """
        Service callback that toggles recording on or off.
        """
        if self.recording:
            self.stop_recording()
            self.recording = False
            response.success = True
            response.message = "Recording stopped."
        else:
            self.start_recording()
            self.recording = True
            response.success = True
            response.message = "Recording started."
        return response

    def topic_callback(self, msg, topic_name: str):
        """
        Callback for subscribed topics. Only writes to the bag if recording is active.
        """
        if self.recording and self.writer is not None:
            self.writer.write(
                topic_name,
                serialize_message(msg),
                self.get_clock().now().nanoseconds
            )


def main(args=None):
    rclpy.init(args=args)
    recorder = DynamicBagRecorder()
    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        pass
    recorder.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
