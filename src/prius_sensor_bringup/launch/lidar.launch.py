from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('prius_sensor_bringup')
    default_params = os.path.join(pkg_share, 'config', 'vlp16.yaml')
    default_rviz = os.path.join(pkg_share, 'rviz', 'vlp16.rviz')

    params_file = LaunchConfiguration('params_file')
    launch_rviz = LaunchConfiguration('rviz')
    rviz_config = LaunchConfiguration('rviz_config')

    lidar_x = LaunchConfiguration('lidar_x')
    lidar_y = LaunchConfiguration('lidar_y')
    lidar_z = LaunchConfiguration('lidar_z')
    lidar_roll = LaunchConfiguration('lidar_roll')
    lidar_pitch = LaunchConfiguration('lidar_pitch')
    lidar_yaw = LaunchConfiguration('lidar_yaw')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='VLP-16 parameter file',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='Launch RViz2',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=default_rviz,
            description='RViz2 config file',
        ),

        # Temporary defaults for bench testing. Replace with measured mounting pose.
        DeclareLaunchArgument('lidar_x', default_value='0.0'),
        DeclareLaunchArgument('lidar_y', default_value='0.0'),
        DeclareLaunchArgument('lidar_z', default_value='0.0'),
        DeclareLaunchArgument('lidar_roll', default_value='0.0'),
        DeclareLaunchArgument('lidar_pitch', default_value='0.0'),
        DeclareLaunchArgument('lidar_yaw', default_value='0.0'),

        Node(
            package='velodyne_driver',
            executable='velodyne_driver_node',
            name='velodyne_driver_node',
            output='screen',
            parameters=[params_file],
        ),

        Node(
            package='velodyne_pointcloud',
            executable='velodyne_transform_node',
            name='velodyne_transform_node',
            output='screen',
            parameters=[params_file],
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='velodyne_static_tf',
            arguments=[
                '--x', lidar_x,
                '--y', lidar_y,
                '--z', lidar_z,
                '--roll', lidar_roll,
                '--pitch', lidar_pitch,
                '--yaw', lidar_yaw,
                '--frame-id', 'base_link',
                '--child-frame-id', 'velodyne',
            ],
            output='screen',
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config, '--fixed-frame', 'base_link'],
            condition=IfCondition(launch_rviz),
            output='screen',
        ),
    ])
