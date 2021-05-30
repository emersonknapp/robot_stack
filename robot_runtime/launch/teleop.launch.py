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
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace


def generate_launch_description():
    runtime_share = get_package_share_directory('robot_runtime')
    cmd_topic = LaunchConfiguration('cmd_topic')

    standard_params = {'use_sim_time': LaunchConfiguration('use_sim_time')}
    teleop_params_file = PathJoinSubstitution([
        runtime_share, 'config', LaunchConfiguration('joy_config')
    ])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('cmd_topic', default_value='/commands/velocity'),
        DeclareLaunchArgument('joy_config', default_value='teleop_8bitdo.yml'),
        # Base
        # Teleop
        Node(
            package='cmd_vel_mux',
            executable='cmd_vel_mux',
            name='cmd_vel_mux',
            parameters=[standard_params],
            remappings=[('/cmd_vel', cmd_topic)],
            output='screen',
        ),
        GroupAction([
            PushRosNamespace('joy'),
            # Node(
            #     package='robot_indicators',
            #     executable='robot_indicators',
            #     name='xpad_led',
            #     parameters=[standard_params],
            #     output='screen',
            # ),
            # Node(
            #     package='robot_indicators',
            #     executable='joy_commands',
            #     name='commands',
            #     parameters=[standard_params],
            #     output='screen',
            # ),
            Node(
                package='joy',
                executable='joy_node',
                name='driver',
                parameters=[standard_params],
                output='screen',
            ),
            Node(
                package='teleop_twist_joy',
                executable='teleop_node',
                name='interpreter',
                parameters=[
                    teleop_params_file,
                    standard_params,
                ],
                output='screen',
            ),
        ]),
    ])
