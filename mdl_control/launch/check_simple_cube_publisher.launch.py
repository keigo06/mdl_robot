#!/usr/bin/env python3

from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():

    publisher_node = Node(
        package='mdl_control',
        executable='simple_cube_publisher.py',
        name='simple_cube_publisher',
        output='screen'
    )

    return LaunchDescription([
        publisher_node,
    ])
