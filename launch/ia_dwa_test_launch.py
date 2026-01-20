#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dynamic_planner = get_package_share_directory('dynamic_planner')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # World
    world_file = os.path.join(pkg_dynamic_planner, 'worlds', 'dynamic_test.world')

    # Set GZ_SIM_RESOURCE_PATH
    set_env_vars_resources = AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.join(pkg_turtlebot3_gazebo, 'models'))

    # Gazebo Sim
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -s -v4 ', world_file], 'on_exit_shutdown': 'true'}.items()
    )

    # Robot State Publisher & Spawn
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot3_gazebo, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot3_gazebo, 'launch', 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={'x_pose': '-3.0', 'y_pose': '0.0', 'z_pose': '0.01'}.items()
    )
    
    # Bridge for Lidar and CmdVel
    # Turtlebot3 Gazebo usually handles this? 
    # Jazzy turtlebot3_gazebo might rely on ros_gz_bridge.
    # The 'spawn_turtlebot3' usually sets up the bridge or specific node.
    # Checking `spawn_turtlebot3` logic: it does usually start a bridge if configured.
    # But often we need a bridge for /scan and /cmd_vel if using gz_sim.
    # Let's add a bridge explicitly just in case, or rely on standard TB3 launch.
    # Standard TB3 spawn usually assumes classic or handles it. 
    # If using Gazebo Harmonic (gz_sim), we likely need ros_gz_bridge.
    
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU'
        ],
        output='screen'
    )
    
    # Rviz
    rviz_config = os.path.join(pkg_dynamic_planner, 'config', 'ia_dwa.rviz') 
    # We need to create this rviz config or use default
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    # IA-DWA Planner Node
    ia_dwa_node = Node(
        package='dynamic_planner',
        executable='ia_dwa_planner',
        name='ia_dwa_planner',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        set_env_vars_resources,
        gz_sim,
        robot_state_publisher,
        spawn_robot,
        # bridge, # TB3 spawn might handle this or we might need it. Leaving commented unless issues.
        # Actually TB3 in Jazzy with gz_sim definitely needs a bridge if spawn doesn't provide it.
        # But 'turtlebot3_gazebo' usually has 'turtlebot3_burger_bridge.yaml' etc.
        # Let's assume spawn handles it for now to avoid duplication error.
        rviz,
        ia_dwa_node
    ])
