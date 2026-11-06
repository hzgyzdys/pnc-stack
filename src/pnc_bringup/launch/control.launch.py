# Copyright (c) 2026 Hu Zhiguo. MIT License.
"""Launch the reference path publisher and the tracking controller.

The simulator is started separately (pnc_sim/launch/sim.launch.py, or
sim_tracking.launch.py for both at once), so this file can be reused unchanged
on the real vehicle.
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
    # Optional second file for the reference path (a test scenario); later
    # files override earlier ones.
    path_params = LaunchConfiguration('path_params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('path_params_file', default_value=default_params),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(
            package='pnc_control',
            executable='reference_path',
            name='reference_path',
            parameters=[params, path_params, {'use_sim_time': use_sim_time}],
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
