"""Start the DH5 controller node.

    ros2 launch dh5_controller controller.launch.py
    ros2 launch dh5_controller controller.launch.py port:=/dev/ttyUSB1
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0',
                              description='Serial port the DH5 is connected to'),
        DeclareLaunchArgument('baud_rate', default_value='115200'),
        DeclareLaunchArgument('modbus_id', default_value='1'),
        DeclareLaunchArgument('publish_period', default_value='0.5',
                              description='Seconds between dh5/AxisInfos messages'),
        Node(
            package='dh5_controller',
            executable='dh5_controller_node',
            name='dh5_controller_node',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'baud_rate': LaunchConfiguration('baud_rate'),
                'modbus_id': LaunchConfiguration('modbus_id'),
                'publish_period': LaunchConfiguration('publish_period'),
            }],
        ),
    ])
