from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


def generate_launch_description():
    pkg_share = FindPackageShare('lidarodom')

    # Launch configurations
    foxglove = LaunchConfiguration('foxglove')
    rviz = LaunchConfiguration('rviz')
    publish_freq = LaunchConfiguration('publish_freq')
    frame_id = LaunchConfiguration('frame_id')
    lvx_config_file = LaunchConfiguration('lvx_config_file')

    # Launch arguments
    declare_foxglove_arg = DeclareLaunchArgument(
        'foxglove',
        default_value='false',
        description='Whether to launch Foxglove Bridge',
        choices=['true', 'false']
    )
    
    declare_rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Whether to launch RViz',
        choices=['true', 'false']
    )

    declare_publish_freq_arg = DeclareLaunchArgument(
        'publish_freq',
        default_value='10.0',
        description='LiDAR publish frequency (5.0, 10.0, 20.0, 50.0)'
    )

    declare_frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='livox_frame',
        description='Frame ID for the LiDAR data'
    )

    declare_lvx_config_file_arg = DeclareLaunchArgument(
        'lvx_config_file',
        default_value=PathJoinSubstitution([pkg_share, 'config', 'MID360_config.json']),
        description='Path to Livox .json configuration file'
    )

    # Livox driver node
    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=[{
            'xfer_format': 0,  # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
            'multi_topic': 0,  # 0-All LiDARs share the same topic, 1-One LiDAR one topic
            'data_src': 0,     # 0-lidar, others-Invalid data src
            'publish_freq': publish_freq,
            'output_data_type': 0,
            'frame_id': frame_id,
            'user_config_path': PathJoinSubstitution([pkg_share, 'config', lvx_config_file]),
        }]
    )

    # Static transform: map -> livox_frame
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'livox_frame']
    )

    # Foxglove Bridge
    foxglove_bridge_pkg = FindPackageShare('foxglove_bridge')
    foxglove_bridge = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            PathJoinSubstitution([foxglove_bridge_pkg, 'launch', 'foxglove_bridge_launch.xml'])
        ),
        condition=IfCondition(foxglove)
    )

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([pkg_share, 'config', 'livox_pointcloud.rviz'])],
        condition=IfCondition(rviz),
        output='screen'
    )

    return LaunchDescription([
        declare_foxglove_arg,
        declare_rviz_arg,
        declare_publish_freq_arg,
        declare_frame_id_arg,
        declare_lvx_config_file_arg,
        static_tf,
        livox_driver,
        foxglove_bridge,
        rviz_node,
    ])
