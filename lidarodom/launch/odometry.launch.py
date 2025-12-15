from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    kiss_icp_package_share = FindPackageShare(package="kiss_icp")

    # Declare launch arguments
    declare_topic_arg = DeclareLaunchArgument(
        "topic",
        description="Pointcloud topic name"
    )
    declare_use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation time"
    )
    declare_visualize_arg = DeclareLaunchArgument(
        "visualize",
        default_value="true",
        description="Enable visualization"
    )
    declare_base_frame_arg = DeclareLaunchArgument(
        "base_frame",
        default_value="",
        description="Base frame (base_link/base_footprint)"
    )
    declare_lidar_odom_frame_arg = DeclareLaunchArgument(
        "lidar_odom_frame",
        default_value="odom_lidar",
        description="Lidar odometry frame"
    )
    declare_config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value="config.yaml",
        description="KISS-ICP configuration file"
    )

    # Launch configurations
    use_sim_time = LaunchConfiguration("use_sim_time")
    pointcloud_topic = LaunchConfiguration("topic")
    visualize = LaunchConfiguration("visualize")
    base_frame = LaunchConfiguration("base_frame")
    lidar_odom_frame = LaunchConfiguration("lidar_odom_frame")
    publish_odom_tf = LaunchConfiguration("publish_odom_tf", default=True)
    invert_odom_tf = LaunchConfiguration("invert_odom_tf", default=True)
    position_covariance = LaunchConfiguration("position_covariance", default=0.1)
    orientation_covariance = LaunchConfiguration("orientation_covariance", default=0.1)
    config_file = LaunchConfiguration("config_file")

    # KISS-ICP node
    kiss_icp_node = Node(
        package="kiss_icp",
        executable="kiss_icp_node",
        name="kiss_icp_node",
        output="screen",
        remappings=[
            ("pointcloud_topic", pointcloud_topic),
        ],
        parameters=[
            {
                # ROS node configuration
                "base_frame": base_frame,
                "lidar_odom_frame": lidar_odom_frame,
                "publish_odom_tf": publish_odom_tf,
                "invert_odom_tf": invert_odom_tf,
                # ROS CLI arguments
                "publish_debug_clouds": visualize,
                "use_sim_time": use_sim_time,
                "position_covariance": position_covariance,
                "orientation_covariance": orientation_covariance,
            },
            PathJoinSubstitution([kiss_icp_package_share, "config", config_file]),
        ],
    )

    message_converter = Node(
        package="lidarodom",
        executable="message_converter",
        name="message_converter",
        output="screen",
    )

    return LaunchDescription([
        declare_topic_arg,
        declare_use_sim_time_arg,
        declare_visualize_arg,
        declare_base_frame_arg,
        declare_lidar_odom_frame_arg,
        declare_config_file_arg,
        kiss_icp_node,
        message_converter,
    ])


