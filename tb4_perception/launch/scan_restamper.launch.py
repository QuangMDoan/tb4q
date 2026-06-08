from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('input_topic', default_value='scan'),
        DeclareLaunchArgument('output_topic', default_value='scan_restamped'),
        DeclareLaunchArgument('max_age_sec', default_value='2.0'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='tb4_perception',
            executable='scan_restamper_node',
            name='scan_restamper',
            output='screen',
            parameters=[{
                'input_topic': LaunchConfiguration('input_topic'),
                'output_topic': LaunchConfiguration('output_topic'),
                'max_age_sec': LaunchConfiguration('max_age_sec'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
        ),
    ])
