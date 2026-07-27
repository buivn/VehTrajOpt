"""Bring up Gazebo Harmonic, spawn diffbot, bridge /clock, spawn controllers."""
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            RegisterEventHandler)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    desc_pkg = FindPackageShare("vto_description")
    sim_pkg = FindPackageShare("vto_simulation")

    # world = a filename in vto_simulation/worlds (e.g. empty.sdf, maze.sdf)
    world = PathJoinSubstitution([sim_pkg, "worlds", LaunchConfiguration("world")])
    bridge_cfg = PathJoinSubstitution([sim_pkg, "config", "bridge.yaml"])
    headless = LaunchConfiguration("headless")

    # robot_description + robot_state_publisher
    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([desc_pkg, "launch", "description.launch.py"])),
        launch_arguments={"use_sim": "true"}.items(),
    )

    gz_launch = PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])

    #GUI run: server + gui
    gz_gui = IncludeLaunchDescription(PythonLaunchDescriptionSource(gz_launch),
                                      launch_arguments={"gz_args": ["-r -v4 ", world]}.items(),
                                      condition=UnlessCondition(headless))
    
    # headless run: server only (-s)
    gz_headless = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch),
            launch_arguments={"gz_args": ["-s -r -v4 ", world]}.items(),
            condition=IfCondition(headless),
    )

    spawn = Node(
        package="ros_gz_sim", executable="create", output="screen",
        arguments=["-topic", "robot_description", "-name", "diffbot", "-z", "0.15",
                   "-x", LaunchConfiguration("x"), "-y", LaunchConfiguration("y")],
    )

    bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge", output="screen",
        parameters=[{"config_file": bridge_cfg}],
    )

    jsb = Node(package="controller_manager", executable="spawner",
               arguments=["joint_state_broadcaster"], output="screen")
    # remap the controller's namespaced topics to the ROS conventions
    # (/cmd_vel, /odom) so teleop, Nav2, and our Phase-1 controllers connect
    # without per-consumer remapping.
    diff = Node(package="controller_manager", executable="spawner",
                arguments=[
                    "diff_drive_controller",
                    "--controller-ros-args", "-r /diff_drive_controller/cmd_vel:=/cmd_vel",
                    "--controller-ros-args", "-r /diff_drive_controller/odom:=/odom",
                ], output="screen")

    # start controllers only after the robot has spawned
    ctrl_after_spawn = RegisterEventHandler(
        OnProcessExit(target_action=spawn, on_exit=[jsb, diff]))

    return LaunchDescription([
        DeclareLaunchArgument("use_sim", default_value="true"),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("world", default_value="empty.sdf"),
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        description, gz_gui, gz_headless, spawn, bridge, ctrl_after_spawn,
    ])
