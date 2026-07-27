"""Phase 1: Gazebo + diffbot + Pure Pursuit following a test path.

  ros2 launch vto_bringup pursuit_sim.launch.py            # with GUI
  ros2 launch vto_bringup pursuit_sim.launch.py headless:=true
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim_pkg = FindPackageShare("vto_simulation")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([sim_pkg, "launch", "gazebo.launch.py"])),
        launch_arguments={"headless": LaunchConfiguration("headless")}.items(),
    )

    pure_pursuit = Node(
        package="vto_control", executable="pure_pursuit", output="screen",
        parameters=[{"use_sim_time": True}],
    )
    path_publisher = Node(
        package="vto_control", executable="path_publisher", output="screen",
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="false"),
        gazebo, pure_pursuit, path_publisher,
    ])
