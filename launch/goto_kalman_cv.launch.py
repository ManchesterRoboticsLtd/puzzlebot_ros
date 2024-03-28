#!/usr/bin/env python3

import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
           
    pkg_puzzlebot_ros = get_package_share_directory('puzzlebot_ros')
    
    # GoTo
    go_to = Node(
        package='puzzlebot_ros',
        executable='goto_point',
        name='go_to_node',
        arguments=[],
        output='screen'
    )
    
    kalman = Node(
        package='puzzlebot_ros',
        executable='kalman_aruco_cv',
        name='kalman_cv_node',
        arguments=[],
        output='screen'
    )
    
    
    aruco = Node(
        package='aruco_opencv',
        executable='aruco_tracker_autostart',
        parameters=[
                {"camera_topic": "/camera"},
                {"camera_info_topic": "/camera_info"},
                {"marker_size": 0.1},
                {"marker_dict": "4X4_50"},
                {"publish_tf": True}
        ],
        remappings=[],
        output='screen'
    )
    
    ld = [aruco, kalman, go_to]

    return LaunchDescription(ld)
