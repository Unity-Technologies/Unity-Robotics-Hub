import glob
import os

from setuptools import setup

package_name = 'unity_robotics_demo'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Unity Robotics',
    maintainer_email='unity-robotics@unity3d.com',
    description='ROS2 Unity Integration Testing',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'color_publisher = unity_robotics_demo.color_publisher:main',
            'position_service = unity_robotics_demo.position_service:main',
        ],
    },
)
