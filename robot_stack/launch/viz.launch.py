#!/usr/bin/env python3
# Copyright 2021 Emerson Knapp
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
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_candy import pkg_share
from launch_ros.actions import Node
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace


def generate_launch_description():
    rviz_config_path = str(pkg_share('robot_stack') / 'config' / 'homey.rviz')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        GroupAction([
            PushRosNamespace('viztools'),
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config_path],
                output='screen',
                parameters=[{
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                }],
            ),
            Node(
                package='rqt_graph',
                executable='rqt_graph',
                name='rqt_graph'
            ),
        ]),
    ])