#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('pkg_17524060'),
        'config',
        'config.yaml'
    )

    return LaunchDescription([
        Node(
            package='pkg_17524060',
            executable='twist_mux.py',
            name='twist_mux',
            output='screen',
        )
    ])