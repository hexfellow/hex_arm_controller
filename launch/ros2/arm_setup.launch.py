#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2024 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2024-07-29
################################################################

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    xpkg_arm = Node(
        name="xnode_arm",
        package="xpkg_arm",
        executable="xnode_arm",
        output="screen",
        parameters=[{
                    "can_device": "vcan0",
                    "arm_series": 1,
#                     "param_overwrite": "joint_data_output.json",
                    }],
        remappings=[
                    ("/xtopic_arm/joint_cmd", "/xtopic_arm/joint_cmd"),
                    ("/xtopic_arm/joint_states", "/joint_states"),
                    ("/xtopic_arm/json_feedback", "/json_feedback"),
                    ])

    return LaunchDescription([xpkg_arm])
