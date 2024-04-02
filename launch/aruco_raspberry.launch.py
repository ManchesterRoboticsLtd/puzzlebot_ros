#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
               
    camera = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='video_source',
        arguments=[],
        parameters=[
        {"video_device": "/dev/video0"},
        {"camera_info_url": "file:///home/puzzlebot/.ros/camera_info/raspy_cam.yaml"},
        {"image_size": [1280, 720]},
        {"pixel_format": "YUYV"},
        {"output_encoding": "rgb8"},
        {"auto_exposure": 1},
        {"exposure_time_absolute": 400}
        ],
        output='screen'
    )
        
    aruco = Node(
        package='aruco_opencv',
        executable='aruco_tracker_autostart',
        parameters=[
                {"cam_base_topic": "/image_raw"},
                {"marker_size": 0.1},
                {"marker_dict": "4X4_50"},
                {"publish_tf": True}
        ],
        remappings=[],
        output='screen'
    )
        
    ld = [camera, aruco]

    return LaunchDescription(ld)
