from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lab1_pkg',
            namespace='talker',
            executable='talker',
            name='talker'
        ),
        Node(
            package='lab1_pkg',
            namespace='relay',
            executable='relay',
            name='relay'
        ),
    ])
