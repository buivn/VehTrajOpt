"""Run the full stack + the benchmark tour for one controller, in one command.

  ros2 launch vto_bench bench.launch.py controller:=pursuit
  ros2 launch vto_bench bench.launch.py controller:=mpc num_legs:=30 repeats:=2

Writes <output_dir>/<controller>_summary.csv + _ticks.csv. Headless by default.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    controller = LaunchConfiguration("controller")
    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare("vto_bringup"), "launch", "maze_astar.launch.py"])),
        launch_arguments={"controller": controller,
                          "headless": LaunchConfiguration("headless")}.items(),
    )
    benchmark = Node(
        package="vto_bench", executable="benchmark", name="benchmark", output="screen",
        parameters=[{
            "use_sim_time": True,
            "label": controller,
            # coerce string launch-args to the types the node declares
            "num_legs": ParameterValue(LaunchConfiguration("num_legs"), value_type=int),
            "repeats": ParameterValue(LaunchConfiguration("repeats"), value_type=int),
            "seed": ParameterValue(LaunchConfiguration("seed"), value_type=int),
            "leg_timeout": ParameterValue(LaunchConfiguration("leg_timeout"), value_type=float),
            "leg_min_dist": ParameterValue(LaunchConfiguration("leg_min_dist"), value_type=float),
            "leg_max_dist": ParameterValue(LaunchConfiguration("leg_max_dist"), value_type=float),
            "output_dir": ParameterValue(LaunchConfiguration("output_dir"), value_type=str),
        }],
    )
    return LaunchDescription([
        DeclareLaunchArgument("controller", default_value="pursuit"),
        DeclareLaunchArgument("headless", default_value="true"),
        DeclareLaunchArgument("num_legs", default_value="30"),
        DeclareLaunchArgument("repeats", default_value="2"),
        DeclareLaunchArgument("seed", default_value="1"),
        DeclareLaunchArgument("leg_timeout", default_value="150.0"),
        DeclareLaunchArgument("leg_min_dist", default_value="10.0"),
        DeclareLaunchArgument("leg_max_dist", default_value="25.0"),
        DeclareLaunchArgument("output_dir", default_value="/workspace/ros2_ws/results"),
        stack, benchmark,
    ])
