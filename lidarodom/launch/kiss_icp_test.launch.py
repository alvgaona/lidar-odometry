from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    kiss_icp_package_share = FindPackageShare(package="kiss_icp")
    lidarodom_package_share = FindPackageShare(package="lidarodom")

    # Declare launch arguments
    declare_bag_file_arg = DeclareLaunchArgument(
        "bag_file",
        description="Path to MCAP rosbag file"
    )
    declare_topic_arg = DeclareLaunchArgument(
        "topic",
        default_value="/livox/lidar",
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
    declare_bag_rate_arg = DeclareLaunchArgument(
        "bag_rate",
        default_value="1.0",
        description="Playback rate for rosbag"
    )
    declare_bag_start_arg = DeclareLaunchArgument(
        "bag_start",
        default_value="0.0",
        description="Start time offset in seconds"
    )

    # Launch configurations
    bag_file = LaunchConfiguration("bag_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    topic = LaunchConfiguration("topic")
    visualize = LaunchConfiguration("visualize")
    base_frame = LaunchConfiguration("base_frame")
    lidar_odom_frame = LaunchConfiguration("lidar_odom_frame")
    config_file = LaunchConfiguration("config_file")
    bag_rate = LaunchConfiguration("bag_rate")
    bag_start = LaunchConfiguration("bag_start")

    # Include KISS-ICP odometry launch file
    kiss_icp_odometry = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([kiss_icp_package_share, "launch", "odometry.launch.py"])
        ),
        launch_arguments={
            "topic": topic,
            "use_sim_time": use_sim_time,
            "visualize": visualize,
            "base_frame": base_frame,
            "lidar_odom_frame": lidar_odom_frame,
            "config_file": config_file,
        }.items()
    )

    # Message converter node
    message_converter = Node(
        package="lidarodom",
        executable="message_converter",
        name="message_converter",
        output="screen",
    )

    # RViz node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", PathJoinSubstitution([lidarodom_package_share, "config", "kiss_icp.rviz"])],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen"
    )

    # Rosbag playback
    rosbag_play = ExecuteProcess(
        cmd=[
            "ros2", "bag", "play", "-s", "mcap",
            bag_file,
            "--clock",
            "--rate", bag_rate,
            "--start-offset", bag_start
        ],
        output="screen"
    )

    return LaunchDescription([
        declare_bag_file_arg,
        declare_topic_arg,
        declare_use_sim_time_arg,
        declare_visualize_arg,
        declare_base_frame_arg,
        declare_lidar_odom_frame_arg,
        declare_config_file_arg,
        declare_bag_rate_arg,
        declare_bag_start_arg,
        kiss_icp_odometry,
        message_converter,
        rviz_node,
        rosbag_play,
    ])
