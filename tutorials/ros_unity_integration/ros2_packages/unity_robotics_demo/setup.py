import glob
import os

from setuptools import setup

package_name = 'ros2_test'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), ['launch/test_launcher.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Unity Robotics',
    maintainer_email='laurie.cheers@unity3d.com',
    description='ROS2 Unity Integration Testing',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'server_endpoint = ros2_test.server_endpoint:main',
            'color_publisher = ros2_test.color_publisher:main',
            'position_service = ros2_test.position_service:main',
        ],
    },
)
