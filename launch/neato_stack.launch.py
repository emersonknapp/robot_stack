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
import pathlib

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    parameters_file_dir = pathlib.Path(__file__).resolve().parent.parent
    parameters_file_path = parameters_file_dir / 'config' / 'teleop_xbox.config.yaml'
    description_launch_path = os.path.join(
        get_package_share_directory('neato_description'),
        'launch', 'description.launch.py')
    cartographer_launch_path = os.path.join(
        get_package_share_directory('robot_stack'),
        'launch', 'cartographer.launch.py')
    rviz_config_path = os.path.join(
        get_package_share_directory('robot_stack'),
        'config', 'homey.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('base_driver', default_value='true'),
        DeclareLaunchArgument('viz', default_value='true'),
        DeclareLaunchArgument('slam', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        Node(
            package='neato_botvac',
            node_executable='neato',
            node_name='neato_base',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            condition=IfCondition(LaunchConfiguration('base_driver'))),
        Node(
            package='joy',
            node_executable='joy_node',
            node_name='joy_driver',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen'),
        Node(
            package='teleop_twist_joy',
            node_executable='teleop_node',
            node_name='joy_interpreter',
            parameters=[
                parameters_file_path,
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
            output='screen'),
        Node(
            package='rviz2',
            node_executable='rviz2',
            node_name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            condition=IfCondition(LaunchConfiguration('viz'))),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch_path),
            launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(cartographer_launch_path),
            launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items(),
            condition=IfCondition(LaunchConfiguration('slam'))),
    ])
