#!/usr/bin/env python3
# Copyright 2020 Emerson Knapp
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
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    stack_share = get_package_share_directory('robot_stack')
    kobuki_description_launch_path = os.path.join(stack_share, 'launch', 'description.launch.py')
    laser_launch_path = os.path.join(
        get_package_share_directory('hls_lfcd_lds_driver'),
        'launch', 'hlds_laser.launch.py')

    use_base_driver = IfCondition(LaunchConfiguration('base_driver'))
    standard_params = {'use_sim_time': LaunchConfiguration('use_sim_time')}

    return LaunchDescription([
        DeclareLaunchArgument('base_driver', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(laser_launch_path),
            launch_arguments=dict({
                'port': '/dev/lds01',
                'frame_id': 'laser_link',
            }, **standard_params).items(),
            condition=use_base_driver,
        ),
        Node(
            package='turtlebot2_drivers',
            node_executable='kobuki_node',
            node_name='kobuki_base',
            parameters=[standard_params],
            condition=use_base_driver,
            output='screen',
        ),
        IncludeLaunchDescription(
                PythonLaunchDescriptionSource(kobuki_description_launch_path),
                launch_arguments={
                    **standard_params,
                    'joint_states': LaunchConfiguration('base_driver')
                }.items(),
        ),
    ])
