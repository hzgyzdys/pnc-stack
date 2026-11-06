# Copyright (c) 2026 Hu Zhiguo. MIT License.
"""Start Gazebo with the Ackermann car and bridge its topics to ROS 2.

ROS topics after launch:
  /odom     nav_msgs/Odometry      ground truth at the rear-axle centre
  /cmd_vel  geometry_msgs/Twist    linear.x = speed, angular.z = yaw rate
  /clock    rosgraph_msgs/Clock    simulation time
  /tf       odom -> base_link

Arguments:
  world:=<path to .sdf>   default: pnc_sim/worlds/s2_circle.sdf
  headless:=true          run the physics server only, no GUI window
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _gazebo(context):
    world = LaunchConfiguration('world').perform(context)
    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'
    # -r: start running immediately; -s: server only; -v 2: warnings and errors.
    gz_args = f"-r {'-s ' if headless else ''}-v 2 {world}"
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items(),
    )]


def generate_launch_description():
    share = get_package_share_directory('pnc_sim')

    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value=os.path.join(share, 'worlds', 's2_circle.sdf')),
        DeclareLaunchArgument('headless', default_value='false'),
        # Lets the world find model://ackermann_car.
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(share, 'models')),
        OpaqueFunction(function=_gazebo),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_bridge',
            parameters=[{'config_file': os.path.join(share, 'config', 'bridge.yaml')}],
            output='screen',
        ),
    ])
