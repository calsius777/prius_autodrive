from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('prius_mti630_driver')
    default_config = os.path.join(pkg_share, 'config', 'mti630.yaml')

    config = LaunchConfiguration('config')
    port = LaunchConfiguration('port')
    baudrate = LaunchConfiguration('baudrate')
    frame_id = LaunchConfiguration('frame_id')

    base_frame = LaunchConfiguration('base_frame')
    imu_frame = LaunchConfiguration('imu_frame')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    roll = LaunchConfiguration('roll')
    pitch = LaunchConfiguration('pitch')
    yaw = LaunchConfiguration('yaw')
    publish_static_tf = LaunchConfiguration('publish_static_tf')

    imu_node = Node(
        package='prius_mti630_driver',
        executable='mti630_node',
        name='mti630_node',
        output='screen',
        parameters=[
            config,
            {
                'port': port,
                'baudrate': baudrate,
                'frame_id': frame_id,
            },
        ],
    )

    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='mti630_static_tf_publisher',
        output='screen',
        arguments=[x, y, z, roll, pitch, yaw, base_frame, imu_frame],
        condition=None,
    )

    # launch conditions cannot be passed as a string directly without extra boilerplate;
    # keep the node always present for the first bringup and set identity transform by default.
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config),
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('baudrate', default_value='115200'),
        DeclareLaunchArgument('frame_id', default_value='mti630_link'),
        DeclareLaunchArgument('publish_static_tf', default_value='true'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('imu_frame', default_value='mti630_link'),
        # Replace these with measured Prius installation extrinsics.
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.0'),
        DeclareLaunchArgument('roll', default_value='0.0'),
        DeclareLaunchArgument('pitch', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        imu_node,
        static_tf_node,
    ])
