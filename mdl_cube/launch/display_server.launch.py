#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    rviz_config_file = os.path.join(
        get_package_share_directory('mdl_cube'),
        'launch',
        'cube_display.rviz'  # Rviz設定ファイルへのパス
    )

    return LaunchDescription([
        # Rvizを起動するノード
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        ),
        # Serverを起動するノード
        Node(
            package='mdl_cube',
            executable='server.py',  # server.pyを実行
            name='server_node',
            output='screen',
        )
    ])
