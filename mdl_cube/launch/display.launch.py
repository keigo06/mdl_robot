#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    rviz_config_file = os.path.join(
        get_package_share_directory('mdl_cube'),
        'launch',
        'cube_display.rviz'
    )

    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        ),
        Node(
            package='mdl_cube',
            executable='cube_node.py',
            name='cube_node',
            output='screen',
        )
    ])
