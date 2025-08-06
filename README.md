# pcl2costmap

This package converts 3D `.pcd` files into 2D map formats such as `.pgm` or `.png`, allowing you to visualize the resulting map in RViz2.

Currently, it only supports occupancy maps. Supporting for costmaps is under development.

By default, the map is generated with a black background, obstacles are represented in white (255), and explored areas are shown in gray (127).

- [Prerequisites](#Prerequisites)
- [How to Use](#how-to-use)


## Development Environment

| Component   | Version          |
|-------------|------------------|
| **OS**      | Ubuntu 22.04     |
| **ROS**     | Humble Hawksbill     |
| **OpenCV**  | 4.5.4            |


## Prerequisites

1) Prepare the 3D point cloud data as a .pcd file.

2) Make sure to install the required package:
```
sudo apt update
sudo apt install ros-humble-nav2-map-server
```

## How to Use
### 1. Convert .pcd to .pgm

1) edit `pcl2costmap/config/params.yaml` to match your environment

`path_of_pcd` : location of your `.pcd` file.

`ground_thresh`,`sky_thresh` : Specify the height thresholds for removing points.

`pgm_name` : desired name for the output .pgm file.


2) After modifying the files, run the following commands.
```
ros2 launch pcl2costmap pcl2costmap_launch.py
```
3) The `origin_x` and `origin_y` values are automatically written to `map.yaml` to ensure that the generated 2D map is spatially aligned with the original 3D point cloud coordinates.

### 2. Visualize map in RViz2:

1) edit the path in both `pcl2costmap/config/map.yaml` and `pcl2costmap/config/loadmap.yaml` files to match your environment.

2) Make sure RViz2 is running and subscribed to the `map` topic before executing the commands below.
```
ros2 run nav2_map_server map_server --ros-args --params-file $(echo ~/pcl2costmap/config/loadmap.yaml)```
```
ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate
```


## more info
You can also generate a 2D image in any format such as `.png`, `.jpg`, or other supported types.