"""Start the DH5 controller node.

    ros2 launch dh5_controller controller.launch.py
    ros2 launch dh5_controller controller.launch.py port:=/dev/ttyUSB1
    ros2 launch dh5_controller controller.launch.py state_rate_hz:=20 sensor_rate_hz:=20
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# (name, default, description) for every argument passed straight through
# to the node as a parameter of the same name.
ARGUMENTS = [
    ('port', '/dev/ttyUSB0', 'Serial port the DH5 is connected to'),
    ('baud_rate', '115200', ''),
    ('modbus_id', '1', ''),
    ('state_rate_hz', '10.0', 'Rate of dh5/AxisInfos and dh5/joint_states'),
    ('sensor_rate_hz', '10.0', 'Rate of dh5/fingertips'),
    ('stale_timeout', '1.0', 'Seconds without any change before a fingertip is flagged stale'),
    ('move_poll_interval', '0.05', 'Seconds between status polls while a move is running'),
    ('contact_threshold', '0.5', 'Default change of (mx, my, fz) that counts as contact'),
    ('move_timeout', '30.0', 'Default timeout of a dh5/move_axes goal, seconds'),
    ('publish_period', '-1.0', 'Deprecated, use state_rate_hz. Seconds between state messages'),
]


def generate_launch_description():
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value=default, description=description)
          for name, default, description in ARGUMENTS),
        Node(
            package='dh5_controller',
            executable='dh5_controller_node',
            name='dh5_controller_node',
            output='screen',
            parameters=[{name: LaunchConfiguration(name) for name, _, _ in ARGUMENTS}],
        ),
    ])
