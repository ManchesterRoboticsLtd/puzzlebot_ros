#!/usr/bin/env python3

import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml
from launch.actions import ExecuteProcess

def generate_launch_description():
           
    gazebo_sim = ExecuteProcess(cmd=['gz', 'sim', 'world_aruco_mip.sdf'], name='gazebo_sim', output='screen')
    
    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/camera@sensor_msgs/msg/Image[gz.msgs.Image',
                   '/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                   #'/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                   #'/model/puzzlebot1/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                   '/ControlR@std_msgs/msg/Float32]gz.msgs.Float',
                   '/ControlL@std_msgs/msg/Float32]gz.msgs.Float',
                   '/VelocityEncR@std_msgs/msg/Float32[gz.msgs.Float',
                   '/VelocityEncL@std_msgs/msg/Float32[gz.msgs.Float'
                  ],
        output='screen'
    )
    
    # Launch a static transform publisher node
    static_transform_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments=["0", "0", "0",    "0", "0", "0",    "base", "puzzlebot1/chassis/camera_sensor",]
    )
    
    ld = [gazebo_sim, bridge, static_transform_camera]

    return LaunchDescription(ld)
