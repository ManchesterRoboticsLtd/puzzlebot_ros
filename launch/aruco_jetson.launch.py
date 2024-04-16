#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
               
    camera = Node(
        package='ros_deep_learning',
        executable='video_source',
        name='video_source',
        arguments=[],
        parameters=[
        {"resource": "csi://0"},
        {"width": 1280},
        {"height": 720},
        {"codec": "unknown"},
        {"loop": 0},
        {"latency": 2000}
        ],
        output='screen'
    )
        
    camera_info = Node(
        package='camera_info_publisher',
        executable='camera_info_publisher',
        name='camera_info_publisher',
        arguments=[],
        parameters=[
                {"camera_calibration_file": "file:///home/puzzlebot/.ros/jetson_cam.yaml"},
                {"camera_info_topic": "/video_source/camera_info"},
                {"frame_id": "camera"}
        ],
        output='screen'
    )
        
    aruco = Node(
        package='aruco_opencv',
        executable='aruco_tracker_autostart',
        parameters=[
                {"cam_base_topic": "/video_source/raw"},
                {"marker_size": 0.1},
                {"marker_dict": "4X4_50"},
                {"publish_tf": True}
        ],
        remappings=[],
        output='screen'
    )
        
    ld = [camera, camera_info, aruco]

    return LaunchDescription(ld)
