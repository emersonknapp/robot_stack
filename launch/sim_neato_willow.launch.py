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
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    gazebo_launch_path = os.path.join(
        get_package_share_directory('neato_gazebo'),
        'launch', 'robot.launch.py')
    stack_launch_path = os.path.join(
        get_package_share_directory('robot_stack'),
        'launch', 'robot_stack.launch.py')

    neato_gazebo_share_dir = get_package_share_directory('neato_gazebo')
    world = os.path.join(neato_gazebo_share_dir, 'worlds', 'neato_test.world')
    neato_share_dir = get_package_share_directory('neato_description')
    description_repo_path = os.path.join(neato_share_dir, '..')

    return LaunchDescription([
        DeclareLaunchArgument(
            'slam', default_value='true'),
        ExecuteProcess(
            cmd=[
                'gazebo', '--verbose', world,
                '-s', 'libgazebo_ros_init.so',
                '-s', 'libgazebo_ros_factory.so',
            ],
            additional_env={
                'GAZEBO_MODEL_PATH': [description_repo_path],
            },
            output='screen',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_path),
            launch_arguments={
                'use_sim_time': 'true',
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(stack_launch_path),
            launch_arguments={
                'base_model': 'neato',
                'base_driver': 'false',
                'viz': 'true',
                'use_sim_time': 'true',
                'slam': LaunchConfiguration('slam'),
                'nav': 'true',
            }.items(),
        ),
    ])
