from launch import LaunchDescription
from launch_ros.actions import Node

# -----------------------------------------------------------------------------
# Launch file for starting the whiteboard drawing demo nodes
# -----------------------------------------------------------------------------
def generate_launch_description():
    """
    Generates the launch description for the whiteboard drawing demo.

    This launch file starts:
    - fixed_frame_tf2_broadcaster: Publishes TF2 frames for the whiteboard and function points.
    - draw_function_whiteboard_node: Controls the robot arm to draw functions on the whiteboard.

    Both nodes are launched with output shown on the screen.
    """
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
