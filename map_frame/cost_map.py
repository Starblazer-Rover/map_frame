import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
import pcl
import numpy as np
from sensor_msgs_py import point_cloud2 as pc2

class OccupancyGridNode(Node):
    def __init__(self):
        super().__init__('occupancy_grid')
        self.subscription = self.create_subscription(PointCloud2, '/depth/PointCloud2', self.timer_callback, 10)
        
        self.publisher = self.create_publisher(OccupancyGrid, '/map/grid', 10)
        self.resolution = 5
        self.grid_size = 50

    def timer_callback(self, msg):
        pcl_data = self.pointcloud2_to_pcl(msg)
        occupancy_grid = self.pcl_to_occupancy_grid(pcl_data, resolution=self.resolution)
        grid_msg = self.create_occupancy_grid_msg(occupancy_grid)
        self.publisher.publish(grid_msg)

    def pointcloud2_to_pcl(self, msg):
        points_list = []

        for point in pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
            points_list.append([point[0], point[1], point[2]])

        
        pcl_data = pcl.PointCloud()
        pcl_data.from_list(points_list)

        return pcl_data
    
    def pcl_to_occupancy_grid(self, pcl_data, resolution):
        grid = np.full((self.grid_size, self.grid_size), -1, dtype=int)

        origin_x = origin_y = -self.grid_size / 2 * resolution

        for point in pcl_data:
            x, y = int((point[0] - origin_x) / resolution), int((point[1] - origin_y) / resolution)

            if 0 <= x < self.grid_size and 0 <= y < self.grid_size:
                grid[x, y] = 100

        return grid.flatten().tolist()
    
    def create_occupancy_grid_msg(self, occupancy_grid):
        msg = OccupancyGrid()
        msg.header.stamp = Clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.info.map_load_time = Clock().now()
        msg.info.resolution = self.resolution
        msg.info.width = self.grid_size
        msg.info.height = self.grid_size
        msg.info.origin.position.x = -self.grid_size / 2 * self.resolution
        msg.info.origin.position.y = -self.grid_size / 2 * self.resolution
        msg.info.origin.position.z - 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = occupancy_grid

        return msg
    

def main(args=None):
    rclpy.init(args=args)
    
    occupancy_grid = OccupancyGridNode()

    try:
        rclpy.spin(occupancy_grid)
    except KeyboardInterrupt:
        pass
    finally:
        occupancy_grid.destroy_node()


if __name__ == '__main__':
    main()