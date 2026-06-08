from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('source_frame', default_value='map'),
        DeclareLaunchArgument('target_frame', default_value='odom'),
        DeclareLaunchArgument('silence_threshold_sec', default_value='0.3'),
        DeclareLaunchArgument('republish_rate_hz', default_value='10.0'),
        DeclareLaunchArgument('max_extrapolation_sec', default_value='5.0'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='tb4_perception',
            executable='tf_keepalive_node',
            name='tf_keepalive',
            output='screen',
            parameters=[{
                'source_frame': LaunchConfiguration('source_frame'),
                'target_frame': LaunchConfiguration('target_frame'),
                'silence_threshold_sec': LaunchConfiguration('silence_threshold_sec'),
                'republish_rate_hz': LaunchConfiguration('republish_rate_hz'),
                'max_extrapolation_sec': LaunchConfiguration('max_extrapolation_sec'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
        ),
    ])
