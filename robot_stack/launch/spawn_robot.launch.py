from launch import LaunchDescription
from launch_candy import (
    pkg_share,
    render_xacro,
)
from launch_ros.actions import (
    Node,
)


def pkg_install(package_name):
    return pkg_share(package_name).parent


def generate_launch_description():
    xacro_path = str(pkg_share('robot_runtime') / 'urdf' / 'homey.urdf.xacro')
    urdf_file = render_xacro(xacro_path)

    ns = '/simulation'

    return LaunchDescription([
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
    ])
