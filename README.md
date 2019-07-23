# dynamic_cloud_detector

[![Build Status](https://travis-ci.org/amslabtech/dynamic_cloud_detector.svg?branch=master)](https://travis-ci.org/amslabtech/dynamic_cloud_detector)

## Overview
TBW

![demo](gif/dynamic_cloud_detector.gif)

## Enviornment
- Ubuntu 16.04 or 18.04
- ROS Kinetic or Melodic

## Requirement
- PCL 1.8
- velodyne_height_map(https://github.com/jack-oquin/velodyne_height_map)

## Install and Build

```
cd catkin_workspace/src
git clone https://github.com/amslabtech/dynamic_cloud_detector.git
cd ..
catkin_make
```

## Nodes
### dynamic_cloud_detector
#### Published topics
- /cloud/static (sensor_msgs/PointCloud2)
  - point cloud on high-occupancy grid
- /cloud/dynamic (sensor_msgs/PointCloud2)
  - point cloud on low-occupancy grid
- ~/occupancy_grid (nav_msgs/OccupancyGrid)
  - for visualization

#### Subscribed topics
- /velodyne_obstacles (sensor_msgs/PointCloud2)
  - point cloud without ground
- /odom/complement (nav_msgs/Odometry)
  - odometry complemented by IMU. raw odometry or estimated pose is also allowed

#### Parameters
- RESOLUTION
  - resolution of occupancy grid map (default: 0.1[m])
- WIDTH
  - widht and height of occupancy grid map (default: 20[m])
- OCCUPANCY_THRESHOLD
  - the grid cell whose occupancy is lower than this threshold is considered as dynamic (default: 0.2)
- BEAM_NUM
  - this node uses beam model for searching free space (default: 720)
- BUFFER_SIZE
  - this parameter defines the buffer size for point cloud and odometry (default: 5)
- SKIP_SCAN_FLAG
  - if this flag is true, this node adds new data to buffer once in twice (default: false)

## How to Use
```
roslaunch dynamic_cloud_detector dynamic_cloud_detector.launch
```
