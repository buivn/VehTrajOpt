"""Phase 0 top-level: Gazebo + diffbot + controllers + rviz.

Drive it:  ros2 run teleop_twist_keyboard teleop_twist_keyboard \
             --ros-args -r cmd_vel:=/diff_drive_controller/cmd_vel_unstamped
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim_pkg = FindPackageShare("vto_simulation")
    desc_pkg = FindPackageShare("vto_description")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([sim_pkg, "launch", "gazebo.launch.py"])),
    )

    rviz = Node(
        package="rviz2", executable="rviz2", output="screen",
        condition=IfCondition(LaunchConfiguration("rviz")),
        arguments=["-d", PathJoinSubstitution([desc_pkg, "rviz", "diffbot.rviz"])],
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true"),
        gazebo, rviz,
    ])
