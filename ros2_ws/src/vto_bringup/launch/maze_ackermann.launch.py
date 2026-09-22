"""Ackermann robot in the maze -- full pipeline (docs 37-39): localization,
global planning, and path following. A separate file from
maze_astar.launch.py (diffbot-specific, untouched).

  ros2 launch vto_bringup maze_ackermann.launch.py                  # full pipeline
  ros2 launch vto_bringup maze_ackermann.launch.py headless:=true   # no GUI
  ros2 launch vto_bringup maze_ackermann.launch.py \
    world_sdf:=maze_rooms.sdf world_map:=maze_rooms.yaml            # 5-room map (docs/43)
  ros2 launch vto_bringup maze_ackermann.launch.py controller:=mpc  # bicycle-model MPC (docs/44)
  ros2 launch vto_bringup maze_ackermann.launch.py planner:=astar   # fast, kinematically-naive baseline (docs/46)
  ros2 launch vto_bringup maze_ackermann.launch.py planner:=follower  # A* + PID/bicycle-model (docs/47-48)

planner:=lattice (default) | astar | follower. docs/31 already established WHY
lattice exists at all -- A* ignores heading and turning radius entirely,
so it can plan a corner this car cannot physically take. planner:=astar
doesn't retract that; it's a deliberate fast baseline (astar_planner is
the SAME unmodified node diffbot uses -- already fully robot-agnostic, no
heading dependency, nothing to adapt) to measure the actual
planning-time/path-quality trade-off against docs/44's finding that the
lattice can take 30+ seconds on a long cross-map leg, rather than assume
one approach is right without measuring the other.

Brings up: gazebo (maze world + ackermannbot + lidar + imu, correct
spawn heading facing the one open corridor) -> ekf_node (owns
odom->base_footprint, fuses /ackermann_steering_controller/odometry + /imu,
docs/37) -> map_server + lifecycle_manager -> amcl (owns map->odom,
docs/38) -> lattice_ackermann_planner (/map + /goal_pose -> /plan, docs/39)
-> pure_pursuit_ackermann (/plan -> /ackermann_steering_controller/reference,
docs/35+39: pose now read via TF map->base_footprint, not raw odometry, to
match /plan's map frame) -> rviz2 (skipped when headless), reusing
maze_astar.rviz unchanged -- its displays (map, /plan, /particle_cloud, TF,
RobotModel off /robot_description) are all robot-agnostic topic/frame
names, not diffbot-specific.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (EqualsSubstitution, LaunchConfiguration,
                                  PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Must match the spawn heading fix: this cell's only clear corridor is north
# (+y) -- yaw=0 (east) and every other cardinal direction hits a wall within
# 0.83 m (verified against maze.sdf's wall geometry).
SPAWN_X = "-9.1"
SPAWN_Y = "-14.1"
SPAWN_YAW = "1.5708"


def generate_launch_description():
    sim_pkg = FindPackageShare("vto_simulation")
    bringup_pkg = FindPackageShare("vto_bringup")
    headless = LaunchConfiguration("headless")
    sim_time = {"use_sim_time": True}

    map_yaml = PathJoinSubstitution([sim_pkg, "maps", LaunchConfiguration("world_map")])
    ekf_cfg = PathJoinSubstitution([bringup_pkg, "config", "ekf_ackermann.yaml"])
    amcl_cfg = PathJoinSubstitution([bringup_pkg, "config", "amcl_ackermann.yaml"])
    rviz_cfg = PathJoinSubstitution([bringup_pkg, "rviz", "maze_astar.rviz"])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([sim_pkg, "launch", "gazebo.launch.py"])),
        launch_arguments={
            "world": LaunchConfiguration("world_sdf"), "robot": "ackermannbot",
            "x": SPAWN_X, "y": SPAWN_Y, "yaw": SPAWN_YAW, "headless": headless,
        }.items(),
    )

    ekf_node = Node(
        package="robot_localization", executable="ekf_node", name="ekf_filter_node",
        output="screen", parameters=[ekf_cfg, sim_time],
    )

    # map_server + lifecycle_manager + amcl: same shape as
    # maze_astar.launch.py's (docs/09, docs/11), new config (docs/38).
    map_server = Node(
        package="nav2_map_server", executable="map_server", name="map_server",
        output="screen", parameters=[sim_time, {"yaml_filename": map_yaml}],
    )
    lifecycle = Node(
        package="nav2_lifecycle_manager", executable="lifecycle_manager",
        name="lifecycle_manager_localization", output="screen",
        parameters=[sim_time, {"autostart": True,
                               "node_names": ["map_server", "amcl"]}],
    )
    amcl = Node(
        package="nav2_amcl", executable="amcl", name="amcl", output="screen",
        parameters=[amcl_cfg],
    )

    planner = LaunchConfiguration("planner")

    # Global planner: planner:=lattice (default, docs/39, state lattice over
    # Dubins primitives, respects turning radius) | astar (docs/46, fast
    # but kinematically-naive baseline -- see module docstring).
    lattice_ackermann_planner = Node(
        package="vto_planning", executable="lattice_ackermann_planner",
        name="lattice_ackermann_planner", output="screen",
        parameters=[sim_time],
        condition=IfCondition(EqualsSubstitution(planner, "lattice")),
    )
    astar_planner = Node(
        package="vto_planning", executable="astar_planner",
        name="astar_planner", output="screen",
        parameters=[sim_time],
        condition=IfCondition(EqualsSubstitution(planner, "astar")),
    )
    # planner:=follower (docs/47-48): A* guidance + PID/bicycle-model
    # trajectory generation, publishing the same /plan the other two do.
    follower_planner = Node(
        package="vto_planning", executable="follower_planner",
        name="follower_planner", output="screen",
        parameters=[sim_time],
        condition=IfCondition(EqualsSubstitution(planner, "follower")),
    )

    controller = LaunchConfiguration("controller")

    # Path follower (docs/35, pose source revised docs/39). controller:=
    # pursuit|mpc (docs/44) -- both publish the same topic, gated so only
    # one is ever active.
    pure_pursuit_ackermann = Node(
        package="vto_control", executable="pure_pursuit_ackermann",
        name="pure_pursuit_ackermann", output="screen",
        parameters=[sim_time],
        condition=IfCondition(EqualsSubstitution(controller, "pursuit")),
    )
    mpc_controller_ackermann = Node(
        package="vto_control", executable="mpc_controller_ackermann",
        name="mpc_controller_ackermann", output="screen",
        parameters=[sim_time],
        condition=IfCondition(EqualsSubstitution(controller, "mpc")),
    )

    # RViz (skipped when headless) -- same config as maze_astar.launch.py's,
    # unchanged: see module docstring for why it needs no ackermann-specific
    # copy.
    rviz = Node(
        package="rviz2", executable="rviz2", name="rviz2", output="screen",
        arguments=["-d", rviz_cfg], parameters=[sim_time],
        condition=UnlessCondition(headless),
    )

    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("world_sdf", default_value="maze.sdf",
                              description="Gazebo world file (vto_simulation/worlds/); "
                                          "maze_rooms.sdf for the 5-room map (docs/43)"),
        DeclareLaunchArgument("world_map", default_value="maze.yaml",
                              description="matching map_server yaml (vto_simulation/maps/)"),
        DeclareLaunchArgument("controller", default_value="pursuit",
                              description="follower: pursuit | mpc (docs/44)"),
        DeclareLaunchArgument("planner", default_value="lattice",
                              description="global planner: lattice | astar (docs/46) | "
                                          "follower (docs/47-48)"),
        gazebo, ekf_node, map_server, lifecycle, amcl,
        lattice_ackermann_planner, astar_planner, follower_planner,
        pure_pursuit_ackermann, mpc_controller_ackermann, rviz,
    ])
