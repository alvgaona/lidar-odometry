# LiDAR Odometry

A repository to run LiDAR odometry algorithms with different LiDAR sensors.

## Build

The ROS 2 packages can easily be built using two options:

1. Standard ROS 2 process using Ubuntu and APT.
2. Using Pixi package manager with Conda repositories.

### APT installation

In this scenario, the general way is to have Ubuntu installed (24.04 recommended).
Then, a ROS workspace is necessary.

```sh
mkdir lidarodom_ws/src
```

After creating the workspace directory, it is just easy as cloning this project.

```sh
git clone --recurse-submodule https://github.com/alvgaona/lidarodom.git
```

The above command clones the repository locally, and pulls the submodules defined in [`.gitmodules`](./.gitmodules). Then, it is just easy as building the packages.

```sh
colcon build --event-handler console_direct+
```

> [!IMPORTANT]
> To build `livox_ros_driver2`, the `Livox-SDK2` must be compiled and installed first; run `cmake --build Livox-SDK2/build && cmake --install Livox-SDK2/build`.

### Pixi installation

In this other scenario, you'd need to have Pixi installed in your system.
Look at this [link](https://pixi.sh/dev/installation/) for instructions.
Then, it is just easy as installing the dependencies defined in [`pixi.toml`](./pixi.toml).

```sh
pixi install
```

This will install the Humble dependencies, but Jazzy dependencies can also be installed by running.

```sh
pixi install -e jazzy
```

After this step, it's easy to build the packages.

```sh
# Builds every package in the workspace
pixi run build
```

Specific package can also be built.

```sh
# Package name must be exactly the one defined in package.xml
pixi run build-pkg <package_name>
```

> [!IMPORTANT]
> To build `livox_ros_driver2`, the `Livox-SDK2` must be compiled and installed first; running `pixi run build-livox-sdk`.

## Usage

Once all is installed, there are a few launch files that can be easily used to visualize some results.

> [!NOTE]
> These examples will be conducted using Pixi but it is the same using using system-wide ROS 2.

Once, you connect your LiDAR to the host machine via the UTP RJ45 cable, just look for the interface in your sysstem, and change its IP address to
192.168.1.50. A simple test is to ping 192.168.1.130 which is the LiDAR.

```text
ping 192.168.1.130
PING 192.168.1.130 (192.168.1.130): 56 data bytes
64 bytes from 192.168.1.130: icmp_seq=0 ttl=255 time=2.336 ms
64 bytes from 192.168.1.130: icmp_seq=1 ttl=255 time=1.842 ms
```

After that, just run this to visualize the point cloud in RViz.
The nodes will take a few seconds to start up and you should see the point cloud in real-time.

```sh
pixi run ros2 launch lidarodom livox.launch.py
```

Then, you'd like to run some LiDAR odometry algorithm, there's this other launch file.

```sh
pixi run ros2 launch lidarodom odometry.launch.py topic:=/livox/points
```

> [!IMPORTANT]
> There's a custom node called `message_converter` that publishes on `/kiss/pose` and `/kiss/path`
> to add more visualization features in RViz.

### Foxglove

Additionally, the `livox.launch.py` launch file has a `foxglove` parameter to spin up the Foxglove bridge to visualize the topics in Foxglove.
It's a bit different but recommended to visualize ROS 2 data.
