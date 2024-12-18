#!/usr/bin/env python3

from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():

    subscriber_node = Node(
        package='mdl_control',
        executable='cube_visualizer.py',
        name='cube_node',
        output='screen'
    )

    return LaunchDescription([
        subscriber_node,
    ])
