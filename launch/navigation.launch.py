#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_dynamic_planner = get_package_share_directory('dynamic_planner')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    params_file = os.path.join(pkg_dynamic_planner, 'config', 'teb_params.yaml')
    
    # Map file - for now using a dummy map or slam?
    # Usually we need a map.yaml. I'll assume we run slam_toolbox or similar if map not provided.
    # Or just run Nav2 without map (AMCL needs map).
    # For simulation, let's use the TurtleBot3 map if available or just slam.
    # User didn't specify map. I will assume SLAM is running or user provides map.
    # For now, let's just launch navigation_launch.py from nav2_bringup with our params
    # But usually that needs a map.
    # If I just want local planner optimization test, maybe I don't need full global map?
    # Let's launch slam_toolbox async if no map is present.
    
    # SIMPLIFICATION: just launch 'navigation_launch.py' and assume user handles mapping or use 'bringup_launch.py' with slam=True
    
    nav2_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'slam': 'True', # Run SLAM to generate map online
            'map': '', # continuous mapping
            'autostart': 'True'
        }.items()
    )

    ld = LaunchDescription()
    ld.add_action(nav2_bringup_cmd)

    return ld
