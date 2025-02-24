import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description(): 
    # log file path
    csv_file_path = 'nav_log.csv'               

    return LaunchDescription([
        Node(
            package='monitor_package',
            executable='monitor_node',
            name='monitor_node',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'csv_filename': csv_file_path},
            ]
        )
    ])
