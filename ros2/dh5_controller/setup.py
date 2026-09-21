from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'dh5_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name, f"{package_name}.clients"],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='ROS 2 controller node for the DH5 hand',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Built on the `dh5` library (pip install it into the ROS env).
            'dh5_controller_node = dh5_controller.controller_node:main',

            'initialize_client = dh5_controller.clients.initialize_client:main',

            'set_position_client = dh5_controller.clients.set_position_client:main',
            'set_speed_client = dh5_controller.clients.set_speed_client:main',
            'set_force_client = dh5_controller.clients.set_force_client:main',

            'clear_cur_fault_client = dh5_controller.clients.clear_cur_fault_client:main',
            'clear_history_faults_client = dh5_controller.clients.clear_history_faults_client:main',
            'get_faults_client = dh5_controller.clients.get_faults_client:main',

            'restart_system_client = dh5_controller.clients.restart_system_client:main',

        ],
    },
)