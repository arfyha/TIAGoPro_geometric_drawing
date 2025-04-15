import os
from os import environ, pathsep
from ament_index_python import get_package_share_directory
from ament_index_python.packages import get_package_prefix

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, SetLaunchConfiguration

from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression

from launch_pal.include_utils import include_scoped_launch_py_description
from launch_pal.actions import CheckPublicSim

from launch_pal.arg_utils import LaunchArgumentsBase
from launch_pal.robot_arguments import CommonArgs
from tiago_pro_description.launch_arguments import TiagoProArgs
from dataclasses import dataclass
from launch_ros.actions import Node


@dataclass(frozen=True)
class LaunchArguments(LaunchArgumentsBase):
    navigation: DeclareLaunchArgument = CommonArgs.navigation
    advanced_navigation: DeclareLaunchArgument = CommonArgs.advanced_navigation
    slam: DeclareLaunchArgument = CommonArgs.slam
    #docking: DeclareLaunchArgument = CommonArgs.docking
    moveit: DeclareLaunchArgument = CommonArgs.moveit
    world_name: DeclareLaunchArgument = CommonArgs.world_name
    tuck_arm: DeclareLaunchArgument = CommonArgs.tuck_arm
    is_public_sim: DeclareLaunchArgument = CommonArgs.is_public_sim


def declare_actions(launch_description: LaunchDescription, launch_args: LaunchArguments):

    tiago_pro_gazebo = include_scoped_launch_py_description(
        pkg_name="tiago_pro_gazebo",
        paths=["launch", "tiago_pro_gazebo.launch.py"],
        launch_arguments={
            'navigation': launch_args.navigation,
            'advanced_navigation': launch_args.advanced_navigation,
            'slam': launch_args.slam,
            #'docking': launch_args.docking,
            'moveit': launch_args.moveit,
            'world_name': "empty",
            'tuck_arm': launch_args.tuck_arm,
            'is_public_sim': launch_args.is_public_sim
        })

    launch_description.add_action(tiago_pro_gazebo)

    moveit_rviz = include_scoped_launch_py_description(
        pkg_name="tiago_pro_moveit_config",
        paths=["launch", "moveit_rviz.launch.py"]
        )

    launch_description.add_action(moveit_rviz)

    whiteboard = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_whiteboard',
            arguments=[
                '-entity', 'whiteboard',
                '-file', '/home/user/exchange/TIAGoPro_WS/src/draw/models/whiteboard.sdf',
                '-x', '1.25',
                '-y', '0',
                '-z', '0',
                '-Y', '1.5708'
            ],
            output='screen'
    )
    launch_description.add_action(whiteboard)

    return


def generate_launch_description():
    # Create the launch description
    ld = LaunchDescription()

    launch_arguments = LaunchArguments()

    launch_arguments.add_to_launch_description(ld)

    declare_actions(ld, launch_arguments)

    return ld