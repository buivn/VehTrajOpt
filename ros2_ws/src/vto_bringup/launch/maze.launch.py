"""Spawn the diffbot inside the 20x30 m maze world.

  ros2 launch vto_bringup maze.launch.py                 # GUI
  ros2 launch vto_bringup maze.launch.py headless:=true

Maze is centered on the world origin (spans ~[-10,10] x [-15,15]); the robot
starts in the bottom-left free cell (~-9.1, -14.1). Drive it with teleop
(see docs/commands.md) or wire Pure Pursuit / Nav2 on top.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim_pkg = FindPackageShare("vto_simulation")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([sim_pkg, "launch", "gazebo.launch.py"])),
        launch_arguments={
            "world": "maze.sdf",
            "x": "-9.1",
            "y": "-14.1",
            "headless": LaunchConfiguration("headless"),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="false"),
        gazebo,
    ])
