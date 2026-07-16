from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('prius_can_interface')
    default_config = os.path.join(pkg_share, 'config', 'prius_can_status.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('interface', default_value='can0'),
        DeclareLaunchArgument('config', default_value=default_config),
        Node(
            package='prius_can_interface',
            executable='prius_can_status_node',
            name='prius_can_status_node',
            output='screen',
            parameters=[
                LaunchConfiguration('config'),
                {'interface': LaunchConfiguration('interface')},
            ],
        ),
    ])
