import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.msg import ParameterType
import yaml

class ParamDumper(Node):
    def __init__(self):
        super().__init__('param_dumper')

        # 🔸 Lista dei nodi da cui vuoi estrarre i parametri
        self.nodes_to_query = ['/attitude/w_ctrl', '/depth/ctrl']  # <-- Sostituisci con i tuoi nodi reali

        # 🔸 Parametri da cercare per ogni nodo (opzionale)
        self.parameters_to_fetch = ['wx_pid_settings', 'wy_pid_settings', 'wz_pid_settings', 'pid_settings']

        # Publisher
        self.pub = self.create_publisher(String, '/parameter_dump', 10)

        # Timer
        self.create_timer(1.0, self.timer_callback)

        # Crea un client per ogni nodo
        self.clients = {}
        for node_name in self.nodes_to_query:
            srv_name = node_name + '/get_parameters'
            client = self.create_client(GetParameters, srv_name)
            self.clients[node_name] = client
            self.get_logger().info(f'Created client for {srv_name}')

    def timer_callback(self):
        for node_name, client in self.clients.items():
            if not client.service_is_ready():
                self.get_logger().warn(f'Service not ready: {node_name}/get_parameters')
                continue

            req = GetParameters.Request()
            req.names = self.parameters_to_fetch

            future = client.call_async(req)
            future.add_done_callback(lambda fut, n=node_name: self.handle_response(n, fut))

    def handle_response(self, node_name, fut):
        if fut.result() is not None:
            values = fut.result().values
            param_dict = {
                node_name: {
                    name: self._convert_value(val)
                    for name, val in zip(self.parameters_to_fetch, values)
                }
            }

            yaml_str = yaml.dump(param_dict, sort_keys=False)
            msg = String()
            msg.data = yaml_str
            self.pub.publish(msg)
        else:
            self.get_logger().error(f'Failed to get parameters from {node_name}')

    def _convert_value(self, param_value):
        if param_value.type == ParameterType.PARAMETER_STRING:
            return param_value.string_value
        elif param_value.type == ParameterType.PARAMETER_INTEGER:
            return param_value.integer_value
        elif param_value.type == ParameterType.PARAMETER_DOUBLE:
            return param_value.double_value
        elif param_value.type == ParameterType.PARAMETER_BOOL:
            return param_value.bool_value
        elif param_value.type == ParameterType.PARAMETER_STRING_ARRAY:
            return list(param_value.string_array_value)
        elif param_value.type == ParameterType.PARAMETER_INTEGER_ARRAY:
            return list(param_value.integer_array_value)
        elif param_value.type == ParameterType.PARAMETER_DOUBLE_ARRAY:
            return list(param_value.double_array_value)
        elif param_value.type == ParameterType.PARAMETER_BOOL_ARRAY:
            return list(param_value.bool_array_value)
        else:
            return None

rclpy.init()
node = ParamDumper()
rclpy.spin(node)
