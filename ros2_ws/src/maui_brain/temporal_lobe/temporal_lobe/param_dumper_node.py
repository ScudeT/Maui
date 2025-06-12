import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import yaml

class ParamDumper(Node):
    def __init__(self):
        super().__init__('param_dumper')
        self.pub = self.create_publisher(String, '/parameter_dump', 10)
        timer_period = 1.0  # once per second
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        params = self._parameters
        yaml_str = yaml.dump({k: v.value for k, v in params.items()})
        msg = String()
        msg.data = yaml_str
        self.pub.publish(msg)

rclpy.init()
node = ParamDumper()
rclpy.spin(node)
