from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('prius_vehicle_status_adapter')
    default_config = os.path.join(pkg_share, 'config', 'prius_vehicle_status_adapter.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config),
        DeclareLaunchArgument('input_topic', default_value='/prius/can/status'),
        Node(
            package='prius_vehicle_status_adapter',
            executable='prius_autoware_vehicle_status_adapter_node',
            name='prius_autoware_vehicle_status_adapter_node',
            output='screen',
            parameters=[
                LaunchConfiguration('config'),
                {'input_topic': LaunchConfiguration('input_topic')},
            ],
        ),
    ])
