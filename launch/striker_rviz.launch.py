import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='px4_ros2_striker',
            executable='striker_action_server',
            name='striker_action_server',
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='px4_ros2_striker',
            executable='rviz_path_publisher',
            name='rviz_path_publisher',
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            # Launch without a specific config initially, the user can configure it and save later
            # arguments=['-d', rviz_config_path]
        )
    ])
