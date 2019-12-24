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
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import Node


def IfEqualsCondition(arg_name, value):
    return IfCondition(PythonExpression([
        '"', LaunchConfiguration(arg_name), '" == "', value, '"'
    ]))


def generate_launch_description():
    stack_share = get_package_share_directory('robot_stack')
    rviz_config_path = os.path.join(stack_share, 'config', 'homey.rviz')
    teleop_params_file = os.path.join(stack_share, 'config', 'teleop_xbox.config.yaml')
    cartographer_launch_path = os.path.join(stack_share, 'launch', 'cartographer.launch.py')
    nav_launch_path = os.path.join(stack_share, 'launch', 'nav.launch.py')
    neato_description_launch_path = os.path.join(
        get_package_share_directory('neato_description'),
        'launch', 'description.launch.py')
    kobuki_description_launch_path = os.path.join(
        get_package_share_directory('robot_stack'),
        'launch', 'description.launch.py')
    laser_launch_path = os.path.join(
        get_package_share_directory('hls_lfcd_lds_driver'),
        'launch', 'hlds_laser.launch.py')

    is_neato_base = IfEqualsCondition('base_model', 'neato')
    is_kobuki_base = IfEqualsCondition('base_model', 'kobuki')
    use_base_driver = IfCondition(LaunchConfiguration('base_driver'))
    standard_params = {'use_sim_time': LaunchConfiguration('use_sim_time')}
    standard_arguments = standard_params.items()

    return LaunchDescription([
        DeclareLaunchArgument('base_model'),
        DeclareLaunchArgument('base_driver', default_value='true'),
        DeclareLaunchArgument('viz', default_value='false'),
        DeclareLaunchArgument('slam', default_value='false'),
        DeclareLaunchArgument('nav', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        # Base
        # -- Neato Base
        GroupAction(
            [
                Node(
                    package='neato_botvac',
                    node_executable='neato',
                    node_name='neato_base',
                    output='screen',
                    parameters=[standard_params],
                    condition=use_base_driver,
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(neato_description_launch_path),
                    launch_arguments=standard_arguments,
                ),
            ],
            condition=is_neato_base,
        ),
        # -- Kobuki Base
        GroupAction(
            [
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
            ],
            condition=is_kobuki_base,
        ),

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

        # Mapping
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(cartographer_launch_path),
            launch_arguments=standard_arguments,
            condition=IfCondition(LaunchConfiguration('slam'))
        ),

        # Navigation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav_launch_path),
            launch_arguments=standard_arguments,
            condition=IfCondition(LaunchConfiguration('nav')),
        ),

        # Visualization
        Node(
            package='rviz2',
            node_executable='rviz2',
            node_name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen',
            parameters=[standard_params],
            condition=IfCondition(LaunchConfiguration('viz')),
        ),
    ])
