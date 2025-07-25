import os
from os import environ, pathsep
from ament_index_python import get_package_share_directory
from ament_index_python.packages import get_package_prefix

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, SetLaunchConfiguration, TimerAction, ExecuteProcess

from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression

from launch_pal.include_utils import include_scoped_launch_py_description
from launch_pal.actions import CheckPublicSim

from launch_pal.arg_utils import LaunchArgumentsBase
from launch_pal.robot_arguments import CommonArgs
from tiago_pro_description.launch_arguments import TiagoProArgs
from dataclasses import dataclass
from launch_ros.actions import Node

# -----------------------------------------------------------------------------
# Launch argument dataclass for configuring simulation and robot options
# -----------------------------------------------------------------------------
@dataclass(frozen=True)
class LaunchArguments(LaunchArgumentsBase):
    navigation: DeclareLaunchArgument = CommonArgs.navigation
    advanced_navigation: DeclareLaunchArgument = CommonArgs.advanced_navigation
    slam: DeclareLaunchArgument = CommonArgs.slam
    docking: DeclareLaunchArgument = CommonArgs.docking
    moveit: DeclareLaunchArgument = CommonArgs.moveit
    world_name: DeclareLaunchArgument = CommonArgs.world_name
    tuck_arm: DeclareLaunchArgument = CommonArgs.tuck_arm
    is_public_sim: DeclareLaunchArgument = CommonArgs.is_public_sim

# -----------------------------------------------------------------------------
# Main function to declare and add all launch actions to the LaunchDescription
# -----------------------------------------------------------------------------
def declare_actions(launch_description: LaunchDescription, launch_args: LaunchArguments):

    # Launch the TIAGo Pro Gazebo simulation with specified arguments
    tiago_pro_gazebo = include_scoped_launch_py_description(
        pkg_name="tiago_pro_gazebo",
        paths=["launch", "tiago_pro_gazebo.launch.py"],
        launch_arguments={
            'navigation': launch_args.navigation,
            'advanced_navigation': launch_args.advanced_navigation,
            'slam': launch_args.slam,
            'docking': launch_args.docking,
            'moveit': launch_args.moveit,
            'world_name': "small_office",
            'tuck_arm': launch_args.tuck_arm,
            'is_public_sim': launch_args.is_public_sim
        })
    launch_description.add_action(tiago_pro_gazebo)

    # Launch MoveIt and RViz for motion planning and visualization
    moveit_rviz = include_scoped_launch_py_description(
        pkg_name="tiago_pro_moveit_config",
        paths=["launch", "moveit_rviz.launch.py"]
        )
    launch_description.add_action(moveit_rviz)

    # -----------------------------------------------------------------------------
    # Spawn the whiteboard model in Gazebo at the specified position and orientation
    # -----------------------------------------------------------------------------
    whiteboard_model_path = os.path.join(
        get_package_share_directory('draw'),
        'models',
        'whiteboard.sdf'
    )

    whiteboard = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_whiteboard',
            arguments=[
                '-entity', 'whiteboard',
                '-file', whiteboard_model_path,
                '-x', '1.85',          # X position in Gazebo world
                '-y', '0',             # Y position in Gazebo world
                '-z', '0',             # Z position in Gazebo world
                '-Y', '1.570796327'    # Yaw orientation in radians
            ],
            output='screen'
    )
    launch_description.add_action(whiteboard)

    # -----------------------------------------------------------------------------
    # Delay the execution of the head_down motion by 30 seconds after launch
    # -----------------------------------------------------------------------------
    head_down = TimerAction(
        period=30.0,
        actions=[
             ExecuteProcess(
                cmd=['ros2', 'run', 'play_motion2', 'run_motion', 'head_down', 'false', '120'],
                output='screen'
        )
        ]
    )
    launch_description.add_action(head_down)

    return

# -----------------------------------------------------------------------------
# Entry point for the launch file, builds and returns the LaunchDescription
# -----------------------------------------------------------------------------
def generate_launch_description():
    # Create the launch description object
    ld = LaunchDescription()

    # Instantiate launch arguments
    launch_arguments = LaunchArguments()

    # Add launch arguments to the launch description
    launch_arguments.add_to_launch_description(ld)

    # Declare and add all actions (simulation, MoveIt, whiteboard, head motion)
    declare_actions(ld, launch_arguments)

    return ld