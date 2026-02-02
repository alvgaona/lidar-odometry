from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


def generate_launch_description():
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
    declare_visualize_arg = DeclareLaunchArgument(
        "visualize",
        default_value="true",
        description="Enable debug cloud visualization"
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
    declare_publish_odom_tf_arg = DeclareLaunchArgument(
        "publish_odom_tf",
        default_value="true",
        description="Publish odometry transform"
    )
    declare_invert_odom_tf_arg = DeclareLaunchArgument(
        "invert_odom_tf",
        default_value="false",
        description="Invert odometry transform"
    )
    declare_position_covariance_arg = DeclareLaunchArgument(
        "position_covariance",
        default_value="0.1",
        description="Position covariance for odometry"
    )
    declare_orientation_covariance_arg = DeclareLaunchArgument(
        "orientation_covariance",
        default_value="0.1",
        description="Orientation covariance for odometry"
    )
    declare_config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value=PathJoinSubstitution([lidarodom_package_share, "config", "kiss_icp.yaml"]),
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
    declare_rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value="false",
        description="Launch RViz visualization"
    )
    declare_foxglove_arg = DeclareLaunchArgument(
        "foxglove",
        default_value="false",
        description="Launch Foxglove bridge"
    )

    # Launch configurations
    bag_file = LaunchConfiguration("bag_file")
    topic = LaunchConfiguration("topic")
    visualize = LaunchConfiguration("visualize")
    base_frame = LaunchConfiguration("base_frame")
    lidar_odom_frame = LaunchConfiguration("lidar_odom_frame")
    publish_odom_tf = LaunchConfiguration("publish_odom_tf")
    invert_odom_tf = LaunchConfiguration("invert_odom_tf")
    position_covariance = LaunchConfiguration("position_covariance")
    orientation_covariance = LaunchConfiguration("orientation_covariance")
    config_file = LaunchConfiguration("config_file")
    bag_rate = LaunchConfiguration("bag_rate")
    bag_start = LaunchConfiguration("bag_start")
    rviz = LaunchConfiguration("rviz")
    foxglove = LaunchConfiguration("foxglove")

    # KISS-ICP odometry node
    kiss_icp_node = Node(
        package="kiss_icp",
        executable="kiss_icp_node",
        name="kiss_icp_node",
        output="screen",
        remappings=[("pointcloud_topic", topic)],
        parameters=[
            {
                "base_frame": base_frame,
                "lidar_odom_frame": lidar_odom_frame,
                "publish_odom_tf": publish_odom_tf,
                "invert_odom_tf": invert_odom_tf,
                "publish_debug_clouds": visualize,
                "position_covariance": position_covariance,
                "orientation_covariance": orientation_covariance,
            },
            config_file,
        ],
    )

    # Message converter node
    message_converter = Node(
        package="lidarodom",
        executable="message_converter",
        name="message_converter",
        output="screen",
    )

    # RViz node (optional)
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", PathJoinSubstitution([lidarodom_package_share, "config", "kiss_icp.rviz"])],
        condition=IfCondition(rviz),
        output="screen",
    )

    # Foxglove bridge (optional)
    foxglove_bridge_pkg = FindPackageShare("foxglove_bridge")
    foxglove_bridge = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            PathJoinSubstitution([foxglove_bridge_pkg, "launch", "foxglove_bridge_launch.xml"])
        ),
        condition=IfCondition(foxglove),
    )

    # Rosbag playback
    rosbag_play = ExecuteProcess(
        cmd=[
            "ros2", "bag", "play", "-s", "mcap",
            bag_file,
            "--rate", bag_rate,
            "--start-offset", bag_start
        ],
        output="screen",
    )

    return LaunchDescription([
        declare_bag_file_arg,
        declare_topic_arg,
        declare_visualize_arg,
        declare_base_frame_arg,
        declare_lidar_odom_frame_arg,
        declare_publish_odom_tf_arg,
        declare_invert_odom_tf_arg,
        declare_position_covariance_arg,
        declare_orientation_covariance_arg,
        declare_config_file_arg,
        declare_bag_rate_arg,
        declare_bag_start_arg,
        declare_rviz_arg,
        declare_foxglove_arg,
        kiss_icp_node,
        message_converter,
        rviz_node,
        foxglove_bridge,
        rosbag_play,
    ])
