from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lab1_pkg',
            namespace='talker',
            executable='talker',
            name='talker'
            parameters=[
            {"v": 6.0},
            {"d": 0.3}
        ),
        Node(
            package='lab1_pkg',
            namespace='relay',
            executable='relay',
            name='relay'
        ),
    ])
