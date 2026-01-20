#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dynamic_planner = get_package_share_directory('dynamic_planner')

    ia_dwa_node = Node(
        package='dynamic_planner',
        executable='ia_dwa_planner',
        name='ia_dwa_planner',
        output='screen',
        parameters=[
            {'use_sim_time': True}
        ]
    )

    ld = LaunchDescription()
    ld.add_action(ia_dwa_node)

    return ld
