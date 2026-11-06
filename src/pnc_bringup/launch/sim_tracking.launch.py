# Copyright (c) 2026 Hu Zhiguo. MIT License.
"""One command for the whole loop: Gazebo + bridge + reference path + controller.

  ros2 launch pnc_bringup sim_tracking.launch.py
  ros2 launch pnc_bringup sim_tracking.launch.py scenario:=s3_narrow_passage
  ros2 launch pnc_bringup sim_tracking.launch.py headless:=true

A scenario is a pair of files with the same name:
  pnc_sim/worlds/<scenario>.sdf                       the world
  pnc_bringup/config/scenarios/<scenario>.yaml        the reference path
Available: s1_straight_offset, s2_circle (default), s3_narrow_passage,
s4_u_turn, s5_static_obstacles, s7_parallel_parking.

The controller part is control.launch.py unchanged, so the same file runs on a
real vehicle without the simulator.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    bringup = get_package_share_directory('pnc_bringup')
    sim = get_package_share_directory('pnc_sim')
    scenario = LaunchConfiguration('scenario')

    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='s2_circle'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument(
            'params_file', default_value=os.path.join(bringup, 'config', 'control.yaml')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(sim, 'launch', 'sim.launch.py')),
            launch_arguments={
                'headless': LaunchConfiguration('headless'),
                'world': PathJoinSubstitution([sim, 'worlds', [scenario, '.sdf']]),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'control.launch.py')),
            launch_arguments={
                'params_file': LaunchConfiguration('params_file'),
                'path_params_file': PathJoinSubstitution(
                    [bringup, 'config', 'scenarios', [scenario, '.yaml']]),
                'use_sim_time': 'true',
            }.items(),
        ),
    ])
