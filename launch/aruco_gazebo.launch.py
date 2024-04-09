#!/usr/bin/env python3

import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
               
    pkg_puzzlebot_ros = get_package_share_directory('puzzlebot_ros')
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_puzzlebot_ros, 'gazebo_aruco_world.launch.py'),
        )
    )
        
    aruco = Node(
        package='aruco_opencv',
        executable='aruco_tracker_autostart',
        parameters=[
                {"cam_base_topic": "/camera"},
                {"marker_size": 0.1},
                {"marker_dict": "4X4_50"},
                {"publish_tf": True}
        ],
        remappings=[],
        output='screen'
    )
        
    ld = [gazebo, aruco]

    return LaunchDescription(ld)
