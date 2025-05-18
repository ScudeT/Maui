import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class DebounceNode(Node):
    def __init__(self):
        super().__init__('debounce_node')
        
        # Declare and read the 'timeout' parameter (default: 5.0 seconds)
        self.declare_parameter('timeout', 10.0)
        self.timeout = self.get_parameter('timeout').value
        self.get_logger().info(f"Timeout set to {self.timeout} seconds.")

        # Using a single topic for both input and output.
        topic_name = 'button_state'
        
        # Create the subscriber to the shared topic.
        self.subscription = self.create_subscription(
            Bool,
            topic_name,
            self.input_callback,
            10)
        
        # Create the publisher for the same shared topic.
        self.publisher_ = self.create_publisher(Bool, topic_name, 10)
        
        # Timer to delay publishing True if no further external True arrives.
        self.timer = None
        
        # Record the time when a True message is published by this node.
        self.last_publish_time = None
        
        # Threshold (in seconds) to consider a received True as our own published message.
        self.own_pub_threshold = 0.1

    def input_callback(self, msg: Bool):
        current_time = self.get_clock().now().nanoseconds * 1e-9  # Convert nanoseconds to seconds
        
        if msg.data:
            # Check if this True message is likely our own publication.
            if self.last_publish_time is not None and (current_time - self.last_publish_time < self.own_pub_threshold):
                self.get_logger().debug("Ignoring self-published True message.")
                return

            self.get_logger().info("Received external True message.")
            
            # If a timer is already running (from a previous external True), cancel it.
            if self.timer is not None:
                self.timer.cancel()
            
            # Start a new one-shot timer with the timeout parameter.
            self.timer = self.create_timer(self.timeout, self.timer_callback)
        else:
            self.get_logger().info("Received False message; no action taken.")

    def timer_callback(self):
        self.get_logger().info("Timeout expired without a new external True. Publishing True.")
        out_msg = Bool(data=True)
        self.publisher_.publish(out_msg)
        
        # Record the publication time to filter out our own message later.
        self.last_publish_time = self.get_clock().now().nanoseconds * 1e-9
        
        # Stop the timer; we only publish once per external True sequence.
        self.timer.cancel()
        self.timer = None

def main(args=None):
    rclpy.init(args=args)
    node = DebounceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
