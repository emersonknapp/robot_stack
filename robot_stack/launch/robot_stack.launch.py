#!/usr/bin/env python3
# Copyright 2019 Emerson Knapp
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_candy import include_launch
from launch_ros.actions import Node


def generate_launch_description():
    stack_share = get_package_share_directory('robot_stack')
    rviz_config_path = os.path.join(stack_share, 'config', 'homey.rviz')
    standard_params = {'use_sim_time': LaunchConfiguration('use_sim_time')}
    map_path = os.path.join(stack_share, 'maps', 'willow-partial0.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='rviz2',
            node_executable='rviz2',
            node_name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen',
            parameters=[standard_params],
        ),
        include_launch(
            'robot_runtime', 'robot_runtime.launch.py',
            launch_arguments={
                **standard_params,
                'base_driver': 'false',
                # 'slam': 'true',
                'nav': 'true',
                'map_path': map_path
            }.items())
    ])
