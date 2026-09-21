import rclpy
from rclpy.node import Node
from dh5_interfaces.srv import SetValues

class SetSpeedClient(Node):
    def __init__(self):
        super().__init__('set_speed_client')

        self.declare_parameter('values', [100, 100, 100, 100, 100, 100])

        self.cli = self.create_client(SetValues, 'dh5/set_speed')
        while not self.cli.wait_for_service(timeout_sec=0.05):
            self.get_logger().info('等待服务 dh5/set_speed...')

    def send_request(self):

        values = self.get_parameter('values').get_parameter_value().integer_array_value

        req = SetValues.Request()

        req.value = list(values)

        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result():
            self.get_logger().info(f'速度设置结果: {future.result().success}, {future.result().message}')
        else:
            self.get_logger().error('调用失败')

def main():
    rclpy.init()
    client = SetSpeedClient()
    client.send_request()
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
