from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_test',
            executable='position_service',
        ),
        Node(
            package='ros2_test',
            executable='server_endpoint',
            emulate_tty=True,
            parameters=[
                {'/ROS_IP': '0.0.0.0'},
                {'/ROS_TCP_PORT': 10000},
            ],
        ),
    ])
    