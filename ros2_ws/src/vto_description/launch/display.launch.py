"""Plain robot-model viewer, no sim: robot_state_publisher +
joint_state_publisher_gui (interactive joint sliders) + RViz. Robot-agnostic
-- works for any robot in urdf/mobile/ via robot:=.

  ros2 launch vto_description display.launch.py robot:=ackermannbot
  ros2 launch vto_description display.launch.py                     # diffbot
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("vto_description")

    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg, "launch", "description.launch.py"])),
        launch_arguments={"use_sim": "false", "robot": LaunchConfiguration("robot")}.items(),
    )

    jsp_gui = Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui",
                   output="screen")

    rviz = Node(package="rviz2", executable="rviz2", output="screen",
                arguments=["-d", PathJoinSubstitution([pkg, "rviz", "display.rviz"])])

    return LaunchDescription([
        DeclareLaunchArgument("robot", default_value="diffbot",
                              description="robot: diffbot | ackermannbot"),
        description, jsp_gui, rviz,
    ])
