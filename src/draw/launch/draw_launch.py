from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
        package='surface_detection',
        executable='surface_detector',
        name='surface_detector',
        #parameters=[{
        #    'use_sim_time': True 
        #}]
    ),
        Node(
            package='draw',
            executable='draw_circle_whiteboard',
            name='draw_circle_whiteboard',
            output='screen',
        )
    ])
