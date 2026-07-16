from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("multi_cam_publisher"))
    config_file = package_share / "config" / "cameras.yaml"

    return LaunchDescription(
        [
            Node(
                package="multi_cam_publisher",
                executable="multi_cam_publisher_node",
                name="multi_cam_publisher",
                output="screen",
                parameters=[str(config_file)],
            )
        ]
    )
