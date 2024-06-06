import sys

from tutorial_interfaces.srv import CostMap

import rclpy
from rclpy.node import Node


class ClientNode(Node):
    
    def __init__(self):
        
        super().__init__('test_client')

        self.cli = self.create_client(CostMap, 'cost_map')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            print('service not available...')

        self.req = CostMap.Request()

    def send_request(self):
        self.future = self.cli.call_async(self.req)

        rclpy.spin_until_future_complete(self, self.future)

        return self.future.result()
    

def main():
    rclpy.init()

    test_client = ClientNode()

    response = test_client.send_request()

    print(response)
    
    test_client.destroy_node()


if __name__ == '__main__':
    main()