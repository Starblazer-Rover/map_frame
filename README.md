# Map Frame

This repository is designed to handle all mapping related tasks

## Map Scripts

### Map:

```bash
ros2 run map_frame map
```

#### Script: cost_map.py

#### Subscribed Topics:

/depth/PointCloud2_raw

#### Subscribed Messages:

sensor_msgs/PointCloud2

#### Published Topics:  

/map/grid_raw

#### Published Messages:

nav_msgs/OccupancyGrid

#### Description:

This script takes the rover's pointcloud and creates an occupancy grid by calculating the standard deviation of each points height and seeing if it is above a specified value to determine if it is traversable.