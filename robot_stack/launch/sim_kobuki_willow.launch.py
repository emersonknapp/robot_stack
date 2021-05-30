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
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_candy import (
    include_launch,
    pkg_share,
    render_xacro,
)
from launch_ros.actions import (
    Node,
    PushRosNamespace,
    SetRemap,
)


def pkg_install(package_name):
    return pkg_share(package_name).parent


def generate_launch_description():
    world = str(pkg_share('robot_stack') / 'worlds' / 'willow.world')

    xacro_path = str(pkg_share('robot_runtime') / 'urdf' / 'homey.urdf.xacro')
    urdf_file = render_xacro(xacro_path)

    map_path = str(pkg_share('robot_stack') / 'maps' / 'willow-partial0.yaml')
    # ns = '/simulation'
    ns = ''

    return LaunchDescription([
        DeclareLaunchArgument('slam', default_value='true'),
        DeclareLaunchArgument('nav', default_value='false'),
        DeclareLaunchArgument('base', default_value='kobuki'),
        DeclareLaunchArgument('viz', default_value='false'),
        SetRemap(f'{ns}/cmd_vel', '/cmd_vel'),
        SetRemap(f'{ns}/odom', '/odom'),
        SetRemap(f'{ns}/scan', '/scan'),

        include_launch(
            'gazebo_ros', 'gazebo.launch.py',
            launch_arguments={
                'world': world,
                'verbose': 'true',
            }.items()),
        Node(
            package='rviz2',
            executable='rviz2',
            name='viz',
            condition=IfCondition(LaunchConfiguration('viz')),
            arguments=[
                '-d', str(pkg_share('robot_stack') / 'config' / 'homey.rviz'),
            ]),
        GroupAction([
            PushRosNamespace(ns),
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='spawn_base',
                output='screen',
                arguments=[
                    '-file', urdf_file.name,
                    '-entity', 'homey',
                    '-robot_namespace', ns,
                    '-spawn_service_timeout', '120',
                ],
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='fake_loco',
                output='screen',
                arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
            ),
        ]),
        include_launch(
            'robot_runtime', 'robot_runtime.launch.py',
            launch_arguments={
                'use_sim_time': 'true',
                'base_driver': 'false',
                # 'slam': 'true',
                'nav': 'false',
                'map_path': map_path
            }.items()),
        Node(
            package='kobuki_auto_docking',
            executable='kobuki_sensor_state_agg_node',
            name='kobuki_sensor_agg',
        ),
    ])
