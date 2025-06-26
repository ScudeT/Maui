#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

class ButtonServiceCaller(Node):
    def __init__(self):
        super().__init__('button_service_caller')
        
        # Declare a parameter for the list of service names.
        # This enforces that the parameter is a string array.
        service_names_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY)
        self.declare_parameter('service_names', ['/service_1'], descriptor=service_names_descriptor)
        self.service_names = self.get_parameter('service_names').value
        self.get_logger().info(f"Configured  names: {self.service_names}")

        # Create a subscription to the "button_state" topic.
        self.subscription = self.create_subscription( 
            Bool,
            'button_state',
            self.button_callback,
            10
        )

        # Create a client for each service name and store them in a dictionary.
        self.service_clients = {}
        for service_name in self.service_names:
            client = self.create_client(Trigger, service_name)
            self.service_clients[service_name] = client

    def button_callback(self, msg):
        # Only act on button press (bool True)
        if msg.data:
            self.get_logger().info("Button press detected, calling services...")
            # For each service, wait for it to be available and then call it.
            for service_name, client in self.service_clients.items():
                if not client.wait_for_service(timeout_sec=5.0):
                    self.get_logger().error(f"Service {service_name} is not available.")
                    continue

                req = Trigger.Request()
                future = client.call_async(req)
                # Capture the current service_name in the lambda callback.
                future.add_done_callback(
                    lambda future, service_name=service_name: self.service_response_callback(future, service_name)
                )

    def service_response_callback(self, future, service_name):
        try:
            result = future.result()
            # Log the feedback from the service as info.
            self.get_logger().info(
                f"Service call to {service_name} returned: success={result.success}, message='{result.message}'"
            )
        except Exception as e:
            self.get_logger().error(f"Service call to {service_name} failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ButtonServiceCaller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
