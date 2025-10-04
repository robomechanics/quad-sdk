import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from treadmill_control.BertecRemoteControl import RemoteControl

class TreadmillControlNode(Node):
    def __init__(self):
        super().__init__('bertec_remote_control')

        # parameters for treadmill connection
        self.declare_parameter('server_ip', '127.0.0.1')
        self.declare_parameter('rpc_port', '6001')
        self.declare_parameter('data_port', '6000')
        self.declare_parameter('client_ip', '127.0.0.1')
        self.declare_parameter('client_port', '5560')

        self.declare_parameter('default_accel', 0.5)  # m/s^2
        self.declare_parameter('default_decel', 0.5)  # m/s^2

        # Connect to treadmill
        self.remote = RemoteControl()
        res = self.remote.start_connection(
            server_ip=self.get_parameter('server_ip').get_parameter_value().string_value,
            rpc_port=self.get_parameter('rpc_port').get_parameter_value().string_value,
            data_port=self.get_parameter('data_port').get_parameter_value().string_value,
            client_ip=self.get_parameter('client_ip').get_parameter_value().string_value,
            client_port=self.get_parameter('client_port').get_parameter_value().string_value
        )
        if res is None or res.get('code',0) != 1:
            self.get_logger().error('Failed to connect to treadmill RPC server!')
        else:
            self.get_logger().info('Connected to treadmill RPC server.')

        # Subscribe to treadmill command topic
        self.sub = self.create_subscription(
            Float64MultiArray,     # msg type
            '/treadmill/cmd',      # topic name (self defined)
            self.cmd_callback,     # callback function
            10                     # QoS history depth
        )

    def cmd_callback(self, msg: Float64MultiArray):
        self.get_logger().info(f"Received treadmill cmd: {msg}")
        data = list(msg.data)
        if len(data) == 2:
            lv, rv = data
            default_accel = self.get_parameter('default_accel').get_parameter_value().double_value
            default_decel = self.get_parameter('default_decel').get_parameter_value().double_value
            la = ra = default_accel
            ld = rd = default_decel
        elif len(data) == 6:
            lv, rv, la, ld, ra, rd = data
        else:
            self.get_logger().error(f"/treadmill/cmd expects 2 or 6 numbers, but received {len(data)}.")
            return
        
        res = self.remote.run_treadmill(
            lv, la, ld, rv, ra, rd
        )
        if res:
            self.get_logger().info(f"Treadmill RPC Response: {res}")
        else:
            self.get_logger().warn("No response from treadmill RPC")

def main(args=None):
    rclpy.init(args=args)
    node = TreadmillControlNode()
    rclpy.spin(node)
    node.remote.stop_connection()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()