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
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.substitutions import LaunchConfiguration
from launch_candy import include_launch
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace


def generate_launch_description():
    base_driver = LaunchConfiguration('base_driver')
    use_sim_time = LaunchConfiguration('use_sim_time')
    standard_params = {'use_sim_time': LaunchConfiguration('use_sim_time')}
    base_device = LaunchConfiguration('base_device')

    return LaunchDescription([
        DeclareLaunchArgument('base_driver', default_value='false'),
        DeclareLaunchArgument('base_device', default_value='/dev/ttyUSB0'),
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
            'robot_runtime', 'description.launch.py',
            launch_arguments={
                **standard_params,
                'joint_states': base_driver,
            }.items()),
        Node(
            package='kobuki_node',
            executable='kobuki_ros_node',
            name='kobuki_node',
            parameters=[standard_params, {
                'device_port': base_device,
            }],
            condition=base_driver,
            output='screen',
        ),
        include_launch(
            'robot_runtime', 'teleop.launch.py',
            launch_arguments={
                'use_sim_time': use_sim_time,
            }.items()),
        # Node(
        #     package='prometheus_exporter',
        #     executable='prometheus_exporter',
        #     name='prometheus_exporter',
        #     parameters=[standard_params],
        #     output='screen',
        # ),

        # GroupAction([
        #     PushRosNamespace('parking'),
        #     include_launch(
        #         'parking', 'parking.launch.py',
        #         launch_arguments={
        #             'map': LaunchConfiguration('map_path'),
        #             'use_sim_time': use_sim_time,
        #         }.items()),
        # ]),

        # include_launch(
        #     'robot_runtime', 'cartographer.launch.py', cond='slam',
        #     launch_arguments={
        #         'use_sim_sime': use_sim_time,
        #         'configuration_basename': 'kobuki_lds_2d.lua',
        #     }.items()),
        include_launch(
            'robot_runtime', 'nav.launch.py', cond='nav',
            launch_arguments={
                'use_sim_time': use_sim_time,
                'map': LaunchConfiguration('map_path'),
            }.items()),
    ])
