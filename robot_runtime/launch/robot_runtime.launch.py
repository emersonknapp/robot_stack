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
from launch.actions import GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_candy import include_launch
from launch_ros.actions import Node


def generate_launch_description():
    runtime_share = get_package_share_directory('robot_runtime')
    teleop_params_file = os.path.join(runtime_share, 'config', 'teleop_xbox.config.yaml')
    map_path = os.path.join(runtime_share, 'maps', 'willow-partial0.yaml')

    use_base_driver = IfCondition(LaunchConfiguration('base_driver'))
    use_sim_time = LaunchConfiguration('use_sim_time')
    standard_params = {'use_sim_time': LaunchConfiguration('use_sim_time')}

    return LaunchDescription([
        DeclareLaunchArgument('base_driver', default_value='true'),
        DeclareLaunchArgument('slam', default_value='false'),
        DeclareLaunchArgument('nav', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('map_path', default_value=map_path),
        # Base
        # -- Kobuki Base
        GroupAction([
            include_launch(
                'hls_lfcd_lds_driver', 'hlds_laser.launch.py', cond='base_driver',
                launch_arguments={
                    'port': '/dev/lds01',
                    'frame_id': 'laser_link',
                    'use_sim_time': use_sim_time,
                }.items()),
            Node(
                package='turtlebot2_drivers',
                node_executable='kobuki_node',
                node_name='kobuki_base',
                parameters=[standard_params],
                condition=use_base_driver,
                output='screen',
            ),
            include_launch(
                'robot_runtime', 'description.launch.py', cond=None,
                launch_arguments={
                    **standard_params,
                    'joint_states': LaunchConfiguration('base_driver')
                }.items())
        ]),

        # Teleop
        Node(
            package='joy',
            node_executable='joy_node',
            node_name='joy_driver',
            parameters=[standard_params],
            output='screen',
        ),
        Node(
            package='teleop_twist_joy',
            node_executable='teleop_node',
            node_name='joy_interpreter',
            parameters=[
                teleop_params_file,
                standard_params,
            ],
            output='screen',
        ),
        Node(
            package='robot_indicators',
            node_executable='robot_indicators',
            node_name='robot_indicators',
            parameters=[standard_params],
            output='screen',
        ),

        include_launch(
            'robot_runtime', 'cartographer.launch.py', cond='slam',
            launch_arguments={
                'use_sim_sime': use_sim_time,
                'configuration_basename': 'kobuki_lds_2d.lua',
            }.items()),
        include_launch(
            'robot_runtime', 'nav.launch.py', cond='nav',
            launch_arguments={
                'use_sim_time': use_sim_time,
                'map': LaunchConfiguration('map_path'),
            }.items()),
        include_launch(
            'parking', 'parking.launch.py',
            launch_arguments={
                'map': LaunchConfiguration('map_path'),
                'use_sim_time': use_sim_time,
            }.items()),
    ])
