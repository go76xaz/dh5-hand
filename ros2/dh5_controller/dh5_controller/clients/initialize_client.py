import rclpy
from rclpy.node import Node
from dh5_interfaces.srv import Initialize

class InitializeClient(Node):
    def __init__(self):
        super().__init__('initialize_client')

        self.declare_parameter('mode', 2)

        self.cli = self.create_client(Initialize, 'dh5/initialize')
        while not self.cli.wait_for_service(timeout_sec=0.05):
            self.get_logger().info('等待服务 dh5/initialize...')

    def send_request(self):
        values = self.get_parameter('mode').get_parameter_value().integer_value

        req = Initialize.Request()
        req.mode = values
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result():
            self.get_logger().info(f'初始化结果: {future.result().success}, {future.result().message}')
        else:
            self.get_logger().error('调用失败')

def main():
    rclpy.init()
    client = InitializeClient()
    client.send_request()
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
