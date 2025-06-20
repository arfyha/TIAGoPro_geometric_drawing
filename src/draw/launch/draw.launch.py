from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
        package='draw',
        executable='fixed_frame_tf2_broadcaster',
        name='fixed_frame_tf2_broadcaster',
        output='screen',
    ),
        Node(
            package='draw',
            executable='draw_function_whiteboard',
            name='draw_function_whiteboard_node',
            output='screen',
        )
    ])
