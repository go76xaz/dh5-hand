import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class GetFaultsClient(Node):
    def __init__(self):
        super().__init__('get_faults_client')
        self.cli = self.create_client(Trigger, 'dh5/get_faults')
        while not self.cli.wait_for_service(timeout_sec=0.05):
            self.get_logger().info('等待服务 dh5/get_faults...')

    def send_request(self):
        req = Trigger.Request()
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result():
            self.get_logger().info(f'故障信息: {future.result().message}')
        else:
            self.get_logger().error('调用失败')

def main():
    rclpy.init()
    client = GetFaultsClient()
    client.send_request()
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
