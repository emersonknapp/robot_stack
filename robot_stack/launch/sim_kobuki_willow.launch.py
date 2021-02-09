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
    ExecuteProcess,
    GroupAction,
)
from launch_candy import (
    # include_launch,
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

    model_path = ':'.join([str(p) for p in [
        # for misc sensors
        pkg_install('robot_runtime'),
        # for kobuki mobile base
        pkg_install('kobuki_description'),
        # for stacked plates
        pkg_install('turtlebot_description'),
    ]])
    print(model_path)
    xacro_path = str(pkg_share('robot_runtime') / 'urdf' / 'homey.urdf.xacro')
    urdf_file = render_xacro(xacro_path)

    xacro_path = str(pkg_share('robot_runtime') / 'urdf' / 'dock.urdf')
    dock_urdf = render_xacro(xacro_path)

    map_path = str(pkg_share('robot_stack') / 'maps' / 'willow-partial0.yaml')
    ns = '/simulation'

    return LaunchDescription([
        DeclareLaunchArgument('slam', default_value='true'),
        DeclareLaunchArgument('nav', default_value='false'),
        DeclareLaunchArgument('base', default_value='kobuki'),
        DeclareLaunchArgument('viz', default_value='true'),
        SetRemap(f'{ns}/cmd_vel', '/cmd_vel'),
        SetRemap(f'{ns}/odom', '/odom'),
        SetRemap(f'{ns}/scan', '/scan'),

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
                ],
            ),
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='spawn_dock',
                output='screen',
                arguments=[
                    '-file', dock_urdf.name,
                    '-entity', 'dock',
                    '-robot_namespace', ns,
                ],
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='dock_position',
                output='screen',
                arguments=['0', '2', '0', '0', '0', '0', 'map', 'kobuki_dock']
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='fake_loco',
                output='screen',
                arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
            ),
        ]),
        # include_launch(
        #     'robot_runtime', 'robot_runtime.launch.py',
        #     launch_arguments={
        #         'use_sim_time': 'true',
        #         'base_driver': 'false',
        #         # 'slam': 'true',
        #         'nav': 'true',
        #         'map_path': map_path
        #     }.items())
    ])
