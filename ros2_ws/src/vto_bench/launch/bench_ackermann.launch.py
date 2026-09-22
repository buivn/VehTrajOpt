"""Run the full Ackermann stack + the benchmark tour, in one command (docs/39).

  ros2 launch vto_bench bench_ackermann.launch.py
  ros2 launch vto_bench bench_ackermann.launch.py num_legs:=5 repeats:=1 label:=ackermann_smoke
  ros2 launch vto_bench bench_ackermann.launch.py label:=ackermann_confirm
  ros2 launch vto_bench bench_ackermann.launch.py controller:=mpc label:=mpc_ackermann  # docs/44
  ros2 launch vto_bench bench_ackermann.launch.py planner:=astar label:=astar_ackermann  # docs/46

Same shape as bench.launch.py: planner:=lattice|astar and
controller:=pursuit|mpc are both gated, same axes as bench.launch.py's own
(planner:=astar|lattice, controller:=pursuit|mpc|mppi). Other real
differences from bench.launch.py, both docs/40 (the maze-collision
investigation):
  - benchmark_node.py's /cmd_vel subscription is remapped here to
    /ackermann_steering_controller/reference (both pursuit_ackermann and
    mpc_controller_ackermann's native, unremapped output topic, docs/34) --
    same TwistStamped type, just a different name than diffbot's remapped
    /cmd_vel.
  - goal_validator:=lattice_ackermann (not A*'s default "astar") + spawn_yaw
    set to this robot's real spawn heading, so the tour builder only ever
    picks goals this forward-only car can actually reach -- A* doesn't know
    about turning radius or heading, so it was accepting goals only
    reachable by reversing out of a dead-end, which this robot can't do.
  - repeats defaults to 1, not 2: a repeat round's first leg goes from
    wherever the tour ended back to its own first goal (near spawn) -- an
    edge _build_tour never validates against ANY reachability or distance
    check (goal_validator included), since it isn't part of the goal
    chain at all. Fixing that edge is future work, not done here.
  - world_sdf/world_map default to maze_rooms.sdf/.yaml, not maze.sdf/.yaml
    (docs/43): 5 large (4.8 m square) open rooms, spread across the map,
    replace the original tight-corridor goal pool. This is the actual fix
    for docs/40's heading-trap: a room large enough to turn in any
    direction means no heading is ever a dead end there, so the existing
    forward-only Dubins search (fast, unchanged, no reverse motion needed)
    is sufficient -- see docs/43 for why this was chosen over teaching the
    planner to reverse.
  - room_centers is set to this map's 5 actual room centers (read from
    maze_rooms.sdf's own header comment -- regenerate that map and update
    this list together, they must match), which switches the tour builder
    into "visit each room exactly once" mode instead of randomly sampling
    -- num_legs/leg_min_dist/leg_max_dist/min_clear_cells are unused in
    this mode (kept as launch args only so overriding world_sdf/world_map
    back to the original maze.sdf still has a working random-sample tour;
    see benchmark_node.py's _build_tour docstring).

Writes <output_dir>/<label>_summary.csv + _ticks.csv. Headless by default.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    label = LaunchConfiguration("label")
    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare("vto_bringup"), "launch", "maze_ackermann.launch.py"])),
        launch_arguments={
            "headless": LaunchConfiguration("headless"),
            "world_sdf": LaunchConfiguration("world_sdf"),
            "world_map": LaunchConfiguration("world_map"),
            "controller": LaunchConfiguration("controller"),
            "planner": LaunchConfiguration("planner"),
        }.items(),
    )
    benchmark = Node(
        package="vto_bench", executable="benchmark", name="benchmark", output="screen",
        remappings=[("/cmd_vel", "/ackermann_steering_controller/reference")],
        parameters=[{
            "use_sim_time": True,
            "label": label,
            "goal_validator": "lattice_ackermann",
            "spawn_yaw": 1.5708,   # must match maze_ackermann.launch.py's SPAWN_YAW
            # maze_rooms.sdf's 5 room centers (its own header comment) --
            # visit each exactly once instead of random sampling (docs/43).
            "room_centers": [7.44, -5.80, -7.44, 10.78, 7.44, 5.80,
                             -5.79, -12.43, -7.44, -2.49],
            # coerce string launch-args to the types the node declares
            "min_clear_cells": ParameterValue(LaunchConfiguration("min_clear_cells"), value_type=int),
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
        DeclareLaunchArgument("label", default_value="pursuit_ackermann",
                              description="output-file tag; override to avoid "
                                          "overwriting a prior run (e.g. mpc_ackermann "
                                          "when controller:=mpc, docs/44)"),
        DeclareLaunchArgument("controller", default_value="pursuit",
                              description="follower: pursuit | mpc (docs/44)"),
        DeclareLaunchArgument("planner", default_value="lattice",
                              description="global planner: lattice | astar (docs/46)"),
        DeclareLaunchArgument("headless", default_value="true"),
        DeclareLaunchArgument("world_sdf", default_value="maze_rooms.sdf",
                              description="5-room map (docs/43); maze.sdf for the "
                                          "original tight-corridor maze"),
        DeclareLaunchArgument("world_map", default_value="maze_rooms.yaml"),
        DeclareLaunchArgument("min_clear_cells", default_value="15",
                              description="room-center clearance is ~21 cells, "
                                          "ordinary corridor's p95 is ~14 -- 15 "
                                          "cleanly restricts goals to room interiors "
                                          "(docs/43); lower this back to ~6 if "
                                          "world_sdf/world_map are overridden to the "
                                          "original maze"),
        DeclareLaunchArgument("num_legs", default_value="30"),
        DeclareLaunchArgument("repeats", default_value="1",
                              description="1, not bench.launch.py's 2 -- see "
                                          "module docstring, docs/40"),
        DeclareLaunchArgument("seed", default_value="1"),
        DeclareLaunchArgument("leg_timeout", default_value="150.0"),
        DeclareLaunchArgument("leg_min_dist", default_value="10.0"),
        DeclareLaunchArgument("leg_max_dist", default_value="25.0"),
        DeclareLaunchArgument("output_dir", default_value="/workspace/ros2_ws/results"),
        stack, benchmark,
    ])
