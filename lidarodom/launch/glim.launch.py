from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config_path = PathJoinSubstitution([
        FindPackageShare("lidarodom"), "config", "glim"
    ])

    declare_config_path_arg = DeclareLaunchArgument(
        "config_path",
        default_value=default_config_path,
        description="Absolute path to GLIM config directory",
    )

    declare_rosbag_arg = DeclareLaunchArgument(
        "rosbag",
        default_value="",
        description="Path to rosbag file (optional)",
    )

    config_path = LaunchConfiguration("config_path")
    rosbag = LaunchConfiguration("rosbag")

    rosbag_provided = PythonExpression(["'", rosbag, "' != ''"])

    glim_node = Node(
        package="glim_ros",
        executable="glim_ros_node",
        name="glim_ros_node",
        output="screen",
        parameters=[{"config_path": config_path}],
        condition=UnlessCondition(rosbag_provided),
    )

    glim_rosbag_node = Node(
        package="glim_ros",
        executable="glim_rosbag",
        name="glim_rosbag",
        output="screen",
        arguments=[rosbag],
        parameters=[{"config_path": config_path}],
        condition=IfCondition(rosbag_provided),
    )

    return LaunchDescription([
        declare_config_path_arg,
        declare_rosbag_arg,
        glim_node,
        glim_rosbag_node,
    ])
