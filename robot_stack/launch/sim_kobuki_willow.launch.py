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
from launch.substitutions import LaunchConfiguration
from launch_candy import include_launch
from launch_candy import render_xacro
from launch_ros.actions import Node


def get_package_install_directory(package_name):
    return os.path.join(get_package_share_directory(package_name), '..')


def generate_launch_description():
    world = os.path.join(
        get_package_share_directory('neato_gazebo'),
        'worlds', 'neato_test.world')

    model_path = ':'.join([
        # for misc sensors
        get_package_install_directory('robot_stack'),
        # for kobuki mobile base
        get_package_install_directory('kobuki_description'),
        # for stacked plates
        get_package_install_directory('turtlebot_description'),
    ])
    print(model_path)
    xacro_path = os.path.join(
        get_package_share_directory('robot_stack'), 'urdf', 'homey.urdf.xacro')
    urdf_file = render_xacro(xacro_path)

    return LaunchDescription([
        DeclareLaunchArgument('slam', default_value='true'),
        DeclareLaunchArgument('nav', default_value='false'),
        DeclareLaunchArgument('base', default_value='kobuki'),
        DeclareLaunchArgument('viz', default_value='true'),
        Node(
            package='gazebo_ros',
            node_executable='spawn_entity.py',
            node_name='spawn_base',
            output='screen',
            arguments=['-file', urdf_file.name, '-entity', 'homey'],
        ),
        ExecuteProcess(
            cmd=[
                'gazebo', '--verbose', world,
                '-s', 'libgazebo_ros_init.so',
                '-s', 'libgazebo_ros_factory.so',
            ],
            additional_env={
                'GAZEBO_MODEL_PATH': [model_path],
            },
            output='screen',
        ),
        include_launch(
            'robot_stack', 'robot_stack.launch.py',
            launch_arguments={
                'base_model': LaunchConfiguration('base'),
                'base_driver': 'false',
                'viz': LaunchConfiguration('viz'),
                'use_sim_time': 'true',
                'slam': LaunchConfiguration('slam'),
                'nav': LaunchConfiguration('nav'),
            }.items(),
        ),
    ])
