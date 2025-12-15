from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


def generate_launch_description():
    # Foxglove Bridge
    foxglove_bridge_pkg = FindPackageShare('foxglove_bridge')
    foxglove_bridge = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            PathJoinSubstitution([foxglove_bridge_pkg, 'launch', 'foxglove_bridge_launch.xml'])
        ),
    )

    return LaunchDescription([
        foxglove_bridge,
    ])


