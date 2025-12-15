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
