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

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_prefix = os.path.join(get_package_share_directory('robot_stack'), 'config')

    return LaunchDescription([
        DeclareLaunchArgument(
            'cartographer_config_dir',
            default_value=config_prefix,
            description='Full path to config file to load'),
        DeclareLaunchArgument(
            'configuration_basename',
            default_value='neato_lds_2d.lua',
            description='Name of lua file for cartographer'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        Node(
            package='cartographer_ros',
            node_executable='cartographer_node',
            node_name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            arguments=[
                '-configuration_directory', LaunchConfiguration('cartographer_config_dir'),
                '-configuration_basename', LaunchConfiguration('configuration_basename'),
                '--undefok=r,params-file,ros-args']),

        DeclareLaunchArgument(
            'resolution',
            default_value='0.05',
            description='Resolution of a grid cell in the published occupancy grid'),
        DeclareLaunchArgument(
            'publish_period_sec',
            default_value='1.0',
            description='OccupancyGrid publishing period'),
        Node(
            package='cartographer_ros',
            node_executable='occupancy_grid_node',
            node_name='occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            arguments=[
                '-resolution', LaunchConfiguration('resolution'),
                '-publish_period_sec', LaunchConfiguration('publish_period_sec'),
                '--undefok=r,params-file,ros-args']),
    ])
