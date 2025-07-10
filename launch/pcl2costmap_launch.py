#!/usr/bin/env python3

import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    pkg_share = get_package_share_directory('pcl2costmap')

    param_file = os.path.join(pkg_share, 'config', 'params.yaml')

    pcl2costmap_node = Node(
        package='pcl2costmap',            
        executable='pcl2costmap',         
        name='pcl2costmap',               
        output='screen',
        parameters=[param_file],
    )

    return LaunchDescription([
        pcl2costmap_node,
    ])
