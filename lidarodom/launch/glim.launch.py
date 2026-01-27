from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    declare_config_path_arg = DeclareLaunchArgument(
        "config_path",
        default_value="",
        description="Absolute path to GLIM config directory",
    )

    config_path = LaunchConfiguration("config_path")

    glim_node = Node(
        package="glim_ros",
        executable="glim_ros_node",
        name="glim_ros_node",
        output="screen",
        parameters=[{"config_path": config_path}],
    )

    return LaunchDescription([
        declare_config_path_arg,
        glim_node,
    ])
