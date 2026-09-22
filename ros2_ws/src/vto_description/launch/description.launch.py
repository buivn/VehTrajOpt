"""Publish a robot's robot_description + robot_state_publisher (no sim).

  robot:=diffbot       differential-drive (default)
  robot:=ackermannbot  car-like, docs 31-33
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("vto_description")
    robot = LaunchConfiguration("robot")
    xacro_file = PathJoinSubstitution(
        [pkg, "urdf", "mobile", [robot, ".urdf.xacro"]])

    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro ", xacro_file, " use_sim:=", LaunchConfiguration("use_sim")]),
            value_type=str)
    }

    return LaunchDescription([
        DeclareLaunchArgument("use_sim", default_value="true"),
        DeclareLaunchArgument("robot", default_value="diffbot",
                              description="robot: diffbot | ackermannbot"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[robot_description, {"use_sim_time": LaunchConfiguration("use_sim")}],
        ),
    ])
