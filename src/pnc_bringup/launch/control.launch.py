# Copyright (c) 2026 Hu Zhiguo. MIT License.
"""Launch the reference path publisher and the tracking controller.

The simulator is started separately (sim.launch.py is added in Phase 1), so
this file can be reused unchanged on the real vehicle.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('pnc_bringup'), 'config', 'control.yaml')

    params = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(
            package='pnc_control',
            executable='reference_path',
            name='reference_path',
            parameters=[params, {'use_sim_time': use_sim_time}],
            output='screen',
        ),
        Node(
            package='pnc_control',
            executable='tracking_controller',
            name='tracking_controller',
            parameters=[params, {'use_sim_time': use_sim_time}],
            output='screen',
        ),
    ])
