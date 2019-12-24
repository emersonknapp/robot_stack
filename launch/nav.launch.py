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
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    nav2_bringup_dir = Path(get_package_share_directory('nav2_bringup'))
    nav2_loco = nav2_bringup_dir / 'launch' / 'nav2_localization_launch.py'
    nav2_nav = nav2_bringup_dir / 'launch' / 'nav2_navigation_launch.py'

    map_path = os.path.join(
        get_package_share_directory('robot_stack'), 'maps', 'willow-partial0.yaml')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(nav2_loco)),
            launch_arguments={
                'namespace': '',
                'map': map_path,
                'use_sim_time': 'true',
                'params_file': str(nav2_bringup_dir / 'params' / 'nav2_params.yaml'),
                'use_lifecycle_mgr': 'false',
                'use_remappings': 'false',
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(nav2_nav)),
            launch_arguments={
                'namespace': '',
                'use_sim_time': 'true',
                'params_file': str(nav2_bringup_dir / 'params' / 'nav2_params.yaml'),
                'use_lifecycle_mgr': 'false',
                'use_remappings': 'false',
                'map_subscribe_transient_local': 'true',
            }.items(),
        ),
        Node(
            package='nav2_lifecycle_manager',
            node_executable='lifecycle_manager',
            node_name='lifecycle_manager',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'autostart': True},
                {'node_names': [
                    'map_server',
                    'amcl',
                    'controller_server',
                    'planner_server',
                    'recoveries_server',
                    'bt_navigator',
                    'waypoint_follower'
                ]}])
    ])
