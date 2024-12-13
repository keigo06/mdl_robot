from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # mdl_control_path = get_package_share_path('mdl_control')
    # rviz_config_path = mdl_control_path / 'config/display_cube.rviz'

    # RVIZ Configuration
    rviz_config_path = os.path.join(
        get_package_share_directory('mdl_cube_description'),
        'rviz',
        'urdf_vis.rviz'
    )
    rviz_arg = DeclareLaunchArgument(
        name='rviz_config', default_value=str(rviz_config_path))

    pkg_dir = get_package_share_directory("mdl_cube_description")
    xacro_path = os.path.join(pkg_dir, "urdf", "mdl_cube.xacro")

    # Switch initial assembly
    set_grid_6_as_initial_assembly = 'true'
    grid_6_arg = DeclareLaunchArgument('grid_6',
                                       default_value=set_grid_6_as_initial_assembly)

    robot_description_command = ['xacro ', xacro_path,
                                 ' grid_6:=',
                                 LaunchConfiguration('grid_6'),
                                 ]

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(robot_description_command)}]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        name='rviz_node',
        arguments=['-d', rviz_config_path]
    )

    cube_node = Node(
        package='mdl_control',
        executable='cube_node',
        output='screen'
    )

    return LaunchDescription([
        grid_6_arg,
        rviz_arg,
        robot_state_publisher_node,
        rviz_node
    ])
