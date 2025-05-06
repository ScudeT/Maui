    #!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import Trigger
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

class DepthServiceCaller(Node):
    def __init__(self):
        super().__init__('depth_service_caller')
        
        # Declare a parameter for the list of Trigger service names.
        service_names_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY)
        self.declare_parameter('service_names', ['/service_1'], descriptor=service_names_descriptor)
        self.service_names = self.get_parameter('service_names').value
        self.get_logger().info(f"Configured service names: {self.service_names}")

        # Declare a parameter for the trigger depth.
        depth_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE)
        self.declare_parameter('trigger_depth', 0.0, descriptor=depth_descriptor)
        self.trigger_depth = self.get_parameter('trigger_depth').value
        self.get_logger().info(f"Configured trigger depth: {self.trigger_depth}")

        # Subscribe to the PoseWithCovarianceStamped topic (adjust topic name as necessary)
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            'pose',
            self.pose_callback,
            10
        )

        # Create a client for each Trigger service.
        self.service_clients = {}
        for service_name in self.service_names:
            client = self.create_client(Trigger, service_name)
            self.service_clients[service_name] = client

        # Initialize state: starting outside of the water.
        self.inside = False

    def pose_callback(self, msg):
        # Extract the depth value from the pose.
        depth = msg.pose.pose.position.z
        self.get_logger().debug(f"Received depth: {depth}")

        # Determine if we are inside the water:
        # (inside if depth > trigger_depth, outside otherwise)
        current_inside = depth > self.trigger_depth

        # If the state has changed, trigger the services.
        if current_inside != self.inside:
            if current_inside:
                self.get_logger().info(f"Transition detected: entered water (depth {depth} > trigger depth {self.trigger_depth}).")
            else:
                self.get_logger().info(f"Transition detected: exited water (depth {depth} <= trigger depth {self.trigger_depth}).")
            
            self.trigger_services()
            self.inside = current_inside

    def trigger_services(self):
        # Iterate over each service client and call the Trigger service.
        for service_name, client in self.service_clients.items():
            if not client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error(f"Service {service_name} is not available.")
                continue

            req = Trigger.Request()
            future = client.call_async(req)
            # Use a lambda to capture the service name for the callback.
            future.add_done_callback(
                lambda future, service_name=service_name: self.service_response_callback(future, service_name)
            )

    def service_response_callback(self, future, service_name):
        try:
            result = future.result()
            self.get_logger().info(
                f"Service call to {service_name} returned: success={result.success}, message='{result.message}'"
            )
        except Exception as e:
            self.get_logger().error(f"Service call to {service_name} failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = DepthServiceCaller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
