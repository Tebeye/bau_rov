from setuptools import setup
import os
from glob import glob

package_name = 'rov_line_tracking'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'models/rov_model'), glob('models/rov_model/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ROV Autonomous Team',
    maintainer_email='info@rov.local',
    description='Autonomous Underwater Vehicle (ROV) Line Tracking & Control Package for ROS 2 Humble',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_node = rov_line_tracking.vision_node:main',
            'control_node = rov_line_tracking.control_node:main',
        ],
    },
)
