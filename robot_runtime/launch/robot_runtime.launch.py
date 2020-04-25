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
from launch_ros.actions import PushRosNamespace


def generate_launch_description():
    runtime_share = get_package_share_directory('robot_runtime')
    teleop_params_file = os.path.join(runtime_share, 'config', 'teleop_xbox.config.yaml')

    use_base_driver = IfCondition(LaunchConfiguration('base_driver'))
    use_sim_time = LaunchConfiguration('use_sim_time')
    standard_params = {'use_sim_time': LaunchConfiguration('use_sim_time')}

    return LaunchDescription([
        DeclareLaunchArgument('base_driver', default_value='true'),
        DeclareLaunchArgument('slam', default_value='false'),
        DeclareLaunchArgument('nav', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('map_path'),
        # Base
        # -- Kobuki Base
        include_launch(
            'hls_lfcd_lds_driver', 'hlds_laser.launch.py', cond='base_driver',
            launch_arguments={
                'port': '/dev/lds01',
                'frame_id': 'laser_link',
                'use_sim_time': use_sim_time,
            }.items()),
        include_launch(
            'robot_runtime', 'description.launch.py', cond=None,
            launch_arguments={
                **standard_params,
                'joint_states': use_base_driver,
            }.items()),
        include_launch(
            'robot_runtime', 'teleop.launch.py',
            launch_arguments={
                'base_driver': use_base_driver,
                'use_sim_time': use_sim_time,
            }.items()),

        GroupAction([
            PushRosNamespace('parking'),
            include_launch(
                'parking', 'parking.launch.py',
                launch_arguments={
                    'map': LaunchConfiguration('map_path'),
                    'use_sim_time': use_sim_time,
                }.items()),
        ]),

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
    ])
