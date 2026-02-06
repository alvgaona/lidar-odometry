from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    declare_recording_name = DeclareLaunchArgument(
        "recording_name",
        default_value="lidar_odometry",
        description="Name for the Rerun recording"
    )

    declare_connection_mode = DeclareLaunchArgument(
        "connection_mode",
        default_value="spawn",
        description="Connection mode: 'spawn', 'connect', or 'save'"
    )

    declare_connect_address = DeclareLaunchArgument(
        "connect_address",
        default_value="rerun+http://127.0.0.1:9876/proxy",
        description="Rerun server address (for connect mode)"
    )

    declare_entity_prefix = DeclareLaunchArgument(
        "entity_prefix",
        default_value="/world",
        description="Prefix for all entity paths"
    )

    declare_pointcloud_topics = DeclareLaunchArgument(
        "pointcloud_topics",
        default_value="['/livox/lidar']",
        description="PointCloud2 topics to subscribe"
    )

    declare_pointcloud_entities = DeclareLaunchArgument(
        "pointcloud_entities",
        default_value="['/lidar/points']",
        description="Entity paths for point clouds"
    )

    declare_odometry_topics = DeclareLaunchArgument(
        "odometry_topics",
        default_value="['/kiss/odometry']",
        description="Odometry topics to subscribe"
    )

    declare_odometry_entities = DeclareLaunchArgument(
        "odometry_entities",
        default_value="['/kiss/odom']",
        description="Entity paths for odometry"
    )

    declare_path_topics = DeclareLaunchArgument(
        "path_topics",
        default_value="['/lidarodom/path', '/lidarodom/ground_truth/path']",
        description="Path topics to subscribe"
    )

    declare_path_entities = DeclareLaunchArgument(
        "path_entities",
        default_value="['/odom/trajectory', '/ground_truth/trajectory']",
        description="Entity paths for paths"
    )

    declare_imu_topics = DeclareLaunchArgument(
        "imu_topics",
        default_value="['/livox/imu']",
        description="IMU topics to subscribe"
    )

    declare_imu_entities = DeclareLaunchArgument(
        "imu_entities",
        default_value="['/imu']",
        description="Entity paths for IMU"
    )

    declare_pose_topics = DeclareLaunchArgument(
        "pose_topics",
        default_value="[]",
        description="PoseStamped topics to subscribe"
    )

    declare_pose_entities = DeclareLaunchArgument(
        "pose_entities",
        default_value="[]",
        description="Entity paths for poses"
    )

    declare_tf_parent_frames = DeclareLaunchArgument(
        "tf_parent_frames",
        default_value="[]",
        description="TF parent frames to track"
    )

    declare_tf_child_frames = DeclareLaunchArgument(
        "tf_child_frames",
        default_value="[]",
        description="TF child frames to track"
    )

    rerun_bridge_node = Node(
        package="rerun_bridge",
        executable="rerun_bridge_node",
        name="rerun_bridge",
        output="screen",
        parameters=[{
            "recording_name": LaunchConfiguration("recording_name"),
            "connection_mode": LaunchConfiguration("connection_mode"),
            "connect_address": LaunchConfiguration("connect_address"),
            "entity_prefix": LaunchConfiguration("entity_prefix"),
            "pointcloud_topics": LaunchConfiguration("pointcloud_topics"),
            "pointcloud_entities": LaunchConfiguration("pointcloud_entities"),
            "odometry_topics": LaunchConfiguration("odometry_topics"),
            "odometry_entities": LaunchConfiguration("odometry_entities"),
            "path_topics": LaunchConfiguration("path_topics"),
            "path_entities": LaunchConfiguration("path_entities"),
            "imu_topics": LaunchConfiguration("imu_topics"),
            "imu_entities": LaunchConfiguration("imu_entities"),
            "pose_topics": LaunchConfiguration("pose_topics"),
            "pose_entities": LaunchConfiguration("pose_entities"),
            "tf_parent_frames": LaunchConfiguration("tf_parent_frames"),
            "tf_child_frames": LaunchConfiguration("tf_child_frames"),
        }],
    )

    return LaunchDescription([
        declare_recording_name,
        declare_connection_mode,
        declare_connect_address,
        declare_entity_prefix,
        declare_pointcloud_topics,
        declare_pointcloud_entities,
        declare_odometry_topics,
        declare_odometry_entities,
        declare_path_topics,
        declare_path_entities,
        declare_imu_topics,
        declare_imu_entities,
        declare_pose_topics,
        declare_pose_entities,
        declare_tf_parent_frames,
        declare_tf_child_frames,
        rerun_bridge_node,
    ])
