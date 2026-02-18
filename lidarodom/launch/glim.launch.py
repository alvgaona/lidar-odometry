from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    lidarodom_package_share = FindPackageShare(package="lidarodom")

    default_config_path = PathJoinSubstitution([
        lidarodom_package_share, "config", "glim"
    ])

    declare_config_path_arg = DeclareLaunchArgument(
        "config_path",
        default_value=default_config_path,
        description="Absolute path to GLIM config directory",
    )

    declare_bag_file_arg = DeclareLaunchArgument(
        "bag_file",
        default_value="",
        description="Path to rosbag file (optional)",
    )

    declare_bag_rate_arg = DeclareLaunchArgument(
        "bag_rate",
        default_value="0.15",
        description="Playback rate for rosbag",
    )

    declare_pose_topic_arg = DeclareLaunchArgument(
        "pose_topic",
        default_value="/glim_ros_node/pose",
        description="Pose topic name",
    )

    declare_odom_frame_arg = DeclareLaunchArgument(
        "odom_frame",
        default_value="odom",
        description="Odometry frame ID",
    )

    declare_rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value="false",
        description="Launch RViz visualization",
    )

    config_path = LaunchConfiguration("config_path")
    bag_file = LaunchConfiguration("bag_file")
    bag_rate = LaunchConfiguration("bag_rate")
    pose_topic = LaunchConfiguration("pose_topic")
    odom_frame = LaunchConfiguration("odom_frame")
    rviz = LaunchConfiguration("rviz")

    bag_provided = PythonExpression(["'", bag_file, "' != ''"])

    glim_node = Node(
        package="glim_ros",
        executable="glim_rosnode",
        name="glim_ros_node",
        output="screen",
        parameters=[
            {"config_path": config_path},
            {"use_sim_time": bag_provided},
        ],
    )

    rosbag_play = ExecuteProcess(
        cmd=[
            "ros2", "bag", "play", "-s", "mcap",
            bag_file,
            "--clock",
            "--rate", bag_rate,
        ],
        output="screen",
        condition=IfCondition(bag_provided),
    )

    glim_converter = Node(
        package="lidarodom",
        executable="glim_converter",
        name="glim_converter",
        output="screen",
        parameters=[{
            "pose_topic": pose_topic,
            "odom_frame": odom_frame,
            "use_sim_time": bag_provided,
        }],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", PathJoinSubstitution([lidarodom_package_share, "config", "glim.rviz"])],
        parameters=[{"use_sim_time": bag_provided}],
        condition=IfCondition(rviz),
        output="screen",
    )

    return LaunchDescription([
        declare_config_path_arg,
        declare_bag_file_arg,
        declare_bag_rate_arg,
        declare_pose_topic_arg,
        declare_odom_frame_arg,
        declare_rviz_arg,
        glim_node,
        rosbag_play,
        glim_converter,
        rviz_node,
    ])
