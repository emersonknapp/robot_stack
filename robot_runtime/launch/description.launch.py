import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def render_xacro(xacro_path):
    urdf_content = xacro.process_file(xacro_path)
    urdf_file = tempfile.NamedTemporaryFile(delete=False)
    rendered_urdf = urdf_content.toprettyxml(indent='  ')
    urdf_file.write(rendered_urdf.encode('utf-8'))
    return urdf_file


def generate_launch_description():
    xacro_path = os.path.join(
        get_package_share_directory('robot_runtime'),
        'urdf', 'homey.urdf.xacro')
    urdf_file = render_xacro(xacro_path)

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('publish_frequency', default_value='5.0'),
        DeclareLaunchArgument('joint_states', default_value='false'),
        Node(
            package='robot_state_publisher',
            node_executable='robot_state_publisher',
            node_name='robot_state_publisher',
            output='screen',
            parameters=[{
                'publish_frequency': LaunchConfiguration('publish_frequency'),
                'use_sim_time': LaunchConfiguration('use_sim_time')}],
            arguments=[urdf_file.name],
        ),
        Node(
            package='joint_state_publisher',
            node_executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            arguments=[urdf_file.name],
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
            condition=IfCondition(LaunchConfiguration('joint_states')),
        ),
    ])
