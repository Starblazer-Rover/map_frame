from billee_bot.srv import CostMap

import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid


class TestService(Node):

    def __init__(self):
        super().__init__('test_service')
        self.srv = self.create_service(CostMap, 'cost_map', self.costmap_callback)

    def costmap_callback(self, request, response):
        msg = OccupancyGrid()

        response.costmap = msg
        
        return response
    

def main(args=None):
    rclpy.init(args=args)

    service = TestService()

    try:
        rclpy.spin(service)
    except KeyboardInterrupt:
        pass
    finally:
        service.destroy_node()


if __name__ == '__main__':
    main()