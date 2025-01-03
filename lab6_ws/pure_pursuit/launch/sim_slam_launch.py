from launch import LaunchDescription
from launch_ros.actions import Node

import os

def generate_launch_description():
    return LaunchDescription([
      
      Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
     ),
     
      Node(
        package='nav2_map_server',
        executable='map_server',
        parameters=[{'yaml_filename': config_dict['bridge']['ros__parameters']['map_path'] + '.yaml'},
                    {'topic': 'map'},
                    {'frame_id': 'map'},
                    {'output': 'screen'},
                    {'use_sim_time': True}]
     ),
    ])
