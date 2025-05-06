#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_srvs.srv import Trigger

class DepthWingAmpPublisher(Node):
    def __init__(self):
        super().__init__('depth_wing_amp_setter')
        
        # Declare parameters with default values.
        self.declare_parameter('surface', 0.0)
        self.declare_parameter('depth', 1.0)
        self.declare_parameter('time_on_surface', 5.0)
        self.declare_parameter('time_on_depth', 5.0)
        self.declare_parameter('wing_amp_surface', 0.0)
        self.declare_parameter('wing_amp_depth', 1.0)
        
        # Retrieve parameters.
        self.surface = self.get_parameter('surface').value
        self.depth = self.get_parameter('depth').value
        self.time_on_surface = self.get_parameter('time_on_surface').value
        self.time_on_depth = self.get_parameter('time_on_depth').value
        self.wing_amp_surface = self.get_parameter('wing_amp_surface').value
        self.wing_amp_depth = self.get_parameter('wing_amp_depth').value
        
        # Create publishers.
        self.depth_publisher = self.create_publisher(Float32, 'set/depth', 10)
        self.wing_amp_publisher = self.create_publisher(Float32, 'set/wingAmp', 10)
        
        # Create a trigger service to reset the state and timer to "surface".
        self.create_service(Trigger, 'reset_to_surface', self.reset_to_surface_callback)
        
        # Initialize state to "surface" and publish initial values.
        self.current_state = 'surface'
        self.publish_values(self.surface, self.wing_amp_surface)
        
        # Start the timer using the surface duration.
        self.timer = self.create_timer(self.time_on_surface, self.timer_callback)
    
    def publish_values(self, depth_value: float, wing_amp_value: float):
        # Publish the depth value.
        depth_msg = Float32()
        depth_msg.data = depth_value
        self.depth_publisher.publish(depth_msg)
        self.get_logger().info(f'Published depth: {depth_value} on set/depth')
        
        # Publish the wing amplitude value.
        wing_msg = Float32()
        wing_msg.data = wing_amp_value
        self.wing_amp_publisher.publish(wing_msg)
        self.get_logger().info(f'Published wing amplitude: {wing_amp_value} on set/wingAmp')
    
    def timer_callback(self):
        # Toggle between "surface" and "depth".
        if self.current_state == 'surface':
            self.current_state = 'depth'
            self.publish_values(self.depth, self.wing_amp_depth)
            # Reset timer for depth duration.
            self.timer.cancel()
            self.timer = self.create_timer(self.time_on_depth, self.timer_callback)
        else:
            self.current_state = 'surface'
            self.publish_values(self.surface, self.wing_amp_surface)
            # Reset timer for surface duration.
            self.timer.cancel()
            self.timer = self.create_timer(self.time_on_surface, self.timer_callback)
    
    def reset_to_surface_callback(self, request, response):
        # Service callback to reset the state and the timer to surface.
        self.get_logger().info("Reset_to_surface trigger called: Resetting state and timer to surface.")
        
        # Set state to "surface" and publish corresponding values.
        self.current_state = 'surface'
        self.publish_values(self.surface, self.wing_amp_surface)
        
        # Reset the timer using surface duration.
        self.timer.cancel()
        self.timer = self.create_timer(self.time_on_surface, self.timer_callback)
        
        response.success = True
        response.message = "State reset to surface and timer reset."
        return response

def main(args=None):
    rclpy.init(args=args)
    node = DepthWingAmpPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
