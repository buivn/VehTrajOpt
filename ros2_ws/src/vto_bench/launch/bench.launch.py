"""Run the full stack + the benchmark tour for one controller, in one command.

  ros2 launch vto_bench bench.launch.py controller:=pursuit
  ros2 launch vto_bench bench.launch.py controller:=mpc num_legs:=30 repeats:=2
  ros2 launch vto_bench bench.launch.py controller:=mpc label:=mpc_headless_confirm
  ros2 launch vto_bench bench.launch.py planner:=lattice label:=lattice_confirm  # docs/22

Writes <output_dir>/<label>_summary.csv + _ticks.csv (label defaults to
`controller`; override it to rerun a controller without overwriting a prior
result). Headless by default.
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
    label = LaunchConfiguration("label")
    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare("vto_bringup"), "launch", "maze_astar.launch.py"])),
        launch_arguments={"controller": controller,
                          "planner": LaunchConfiguration("planner"),
                          "headless": LaunchConfiguration("headless"),
                          "mpc_v_min": LaunchConfiguration("mpc_v_min")}.items(),
    )
    benchmark = Node(
        package="vto_bench", executable="benchmark", name="benchmark", output="screen",
        parameters=[{
            "use_sim_time": True,
            "label": label,
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
        DeclareLaunchArgument("planner", default_value="astar",
                              description="global planner: astar | lattice (docs/22)"),
        DeclareLaunchArgument("label", default_value=LaunchConfiguration("controller"),
                              description="output-file tag; defaults to controller "
                                          "name, override to avoid overwriting a "
                                          "prior run with the same controller"),
        DeclareLaunchArgument("headless", default_value="true"),
        DeclareLaunchArgument("mpc_v_min", default_value="0.0"),
        DeclareLaunchArgument("num_legs", default_value="30"),
        DeclareLaunchArgument("repeats", default_value="2"),
        DeclareLaunchArgument("seed", default_value="1"),
        DeclareLaunchArgument("leg_timeout", default_value="150.0"),
        DeclareLaunchArgument("leg_min_dist", default_value="10.0"),
        DeclareLaunchArgument("leg_max_dist", default_value="25.0"),
        DeclareLaunchArgument("output_dir", default_value="/workspace/ros2_ws/results"),
        stack, benchmark,
    ])
