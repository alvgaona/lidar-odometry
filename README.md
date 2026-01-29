# LiDAR Odometry

Run LiDAR odometry algorithms (KISS-ICP) with Livox sensors.

## Quick Start (Pixi)

```sh
pixi install
pixi run build-livox-sdk  # Required for Livox driver
pixi run build
```

## Launch Files

| Launch | Description |
|--------|-------------|
| `livox.launch.py` | Livox point cloud visualization |
| `kiss_icp.launch.py` | KISS-ICP odometry with rosbag |

```sh
# Visualize Livox point cloud
pixi run ros2 launch lidarodom livox.launch.py

# Run KISS-ICP odometry
pixi run ros2 launch lidarodom kiss_icp.launch.py bag_file:=/path/to/bag
```

## Livox Setup

Set host IP to `192.168.1.50`, LiDAR is at `192.168.1.130`.

## Alternative: APT Installation

Standard ROS2 colcon workflow on Ubuntu 24.04. See [.gitmodules](./.gitmodules) for dependencies.
