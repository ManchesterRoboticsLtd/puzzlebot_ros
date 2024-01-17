#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
               
    micro_ros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        arguments=["serial", "-D", "/dev/ttyUSB0"],
        parameters=[],
        output='screen'
    )
                
    ld = [micro_ros_agent]

    return LaunchDescription(ld)
