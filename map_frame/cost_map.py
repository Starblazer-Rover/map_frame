import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from nav_msgs.msg import OccupancyGrid, MapMetaData
import numpy as np
import pcl
from octomap_msgs.msg import Octomap

class CostMap(Node):
class CostMap(Node):
    def __init__(self):
        super().__init__('cost_map')
        self.subscription = self.create_subscription(PointCloud2, '/depth/PointCloud2_raw', self.timer_callback, 10)

        self.publisher = self.create_publisher(OccupancyGrid, '/map/grid_raw', 1)

    def create_info(self):
        map = MapMetaData()

        map.map_load_time = Clock().now().to_msg()
        map.resolution = 24.0
        map.width = 15
        map.height = 15

        map.origin.position.x = 0.0
        map.origin.position.y = -map.resolution * map.width/2
        map.origin.position.z = 0.0
        map.origin.orientation.w = 1.0

        return map
    
    def read_points(self, msg):
        # Read points returns an iterable; we need to convert it to a list to process it
        points_list = list(pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=False))

        # Convert list to a numpy array of shape (-1, 3), inferring the number of rows automatically
        points_array = np.array(points_list, dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4')])

        # Now convert the structured array to a regular float32 array by viewing the data under a new dtype
        points_array = points_array.view((np.float32, 3)).reshape(-1, 3)

        # You might need to adjust the reshaping dimensions to fit your specific case, here assuming 480x640x3
        points_array = points_array.reshape(480, 640, 3)

        return points_array
    
    def find_points(self, column, row):
        #column 7 = (-18.3, 5.8) + 24.0
        #row 2 = (26.0, 52.2) + 23.0
        column_start = (-18.3, 5.8)
        row_start = (26.0, 52.2)

        column_offset = 24.0
        row_offset = 23.0

        column_point = 7
        row_point = 2

        new_column = (column_start[0] + (column - column_point)*column_offset, column_start[1] + (column - column_point)*column_offset)
        new_row = (row_start[0] + (row - row_point)*row_offset, row_start[1] + (row - row_point)*row_offset)

        return new_column, new_row

    def timer_callback(self, msg):

        data = self.read_points(msg)

        grid = OccupancyGrid()
        grid.header = msg.header
        grid.header.frame_id = 'chassis'
        grid.info = self.create_info()

        grid.data = [-1]*(grid.info.height * grid.info.width)

        for i in range(grid.info.height * grid.info.width):
            column = i // grid.info.height
            row = i % grid.info.height

            x_offset, y_offset = self.find_points(column, row)

            indices = np.where((data[:,:,1]>=x_offset[0]) & (data[:,:,1]<=x_offset[1]) & (data[:,:,0]>=y_offset[0]) & (data[:,:,0]<=y_offset[1]))

            if len(indices[0]) != 0:
                deviation = np.std(data[indices[0],indices[1],2])

                if deviation < 4:
                    grid.data[i] = 0
                else:
                    grid.data[i] = min(int(deviation*5), 100)

        """
        grid.data[91] = 100
        grid.data[92] = 100
        grid.data[93] = 100
        grid.data[94] = 100

        grid.data[106] = 100
        grid.data[107] = 100
        grid.data[108] = 100
        grid.data[109] = 100

        grid.data[121] = 100
        grid.data[122] = 100
        grid.data[123] = 100
        grid.data[124] = 100
        """

        print('working')

        self.publisher.publish(grid)
        

        data = self.read_points(msg)

        grid = OccupancyGrid()
        grid.header = msg.header
        grid.header.frame_id = 'chassis'
        grid.info = self.create_info()

        grid.data = [-1]*(grid.info.height * grid.info.width)

        for i in range(grid.info.height * grid.info.width):
            column = i // grid.info.height
            row = i % grid.info.height

            x_offset, y_offset = self.find_points(column, row)

            indices = np.where((data[:,:,1]>=x_offset[0]) & (data[:,:,1]<=x_offset[1]) & (data[:,:,0]>=y_offset[0]) & (data[:,:,0]<=y_offset[1]))

            if len(indices[0]) != 0:
                deviation = np.std(data[indices[0],indices[1],2])

                if deviation < 4:
                    grid.data[i] = 0
                else:
                    grid.data[i] = min(int(deviation*5), 100)

        """
        grid.data[91] = 100
        grid.data[92] = 100
        grid.data[93] = 100
        grid.data[94] = 100

        grid.data[106] = 100
        grid.data[107] = 100
        grid.data[108] = 100
        grid.data[109] = 100

        grid.data[121] = 100
        grid.data[122] = 100
        grid.data[123] = 100
        grid.data[124] = 100
        """

        print('working')

        self.publisher.publish(grid)
        

def main(args=None):
    rclpy.init(args=args)

    cost_map = CostMap()

    cost_map = CostMap()

    try:
        rclpy.spin(cost_map)
        rclpy.spin(cost_map)
    except KeyboardInterrupt:
        pass
    finally:
        cost_map.destroy_node()
        cost_map.destroy_node()


if __name__ == '__main__':
    main()



