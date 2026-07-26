"""Phase 2: solve the maze with A* + Pure Pursuit + AMCL localization.

  ros2 launch vto_bringup maze_astar.launch.py                    # Pure Pursuit + RViz
  ros2 launch vto_bringup maze_astar.launch.py controller:=mpc    # MPC follower
  ros2 launch vto_bringup maze_astar.launch.py headless:=true     # no GUI/RViz

Brings up, in one shot:
  gazebo (maze world + diffbot + lidar) -> map_server (serves maze.yaml on /map)
  -> lifecycle_manager (activates map_server + amcl) -> amcl (owns map->odom)
  -> astar_planner (/map + /goal_pose -> /plan) -> pure_pursuit (/plan -> /cmd_vel)
  -> RViz.

Usage once up: in RViz, click "2D Nav Goal" and pick a spot in the maze.

Notes (see docs/09, docs/11):
  * map_server and amcl are *lifecycle nodes* — lifecycle_manager (autostart)
    configure->activates them in order (map_server first, so amcl has a /map).
  * AMCL now owns the live `map->odom` transform (replacing the old static one).
    For it to actually help, the planner + follower must reference the `map`
    frame via TF (not the old odom-frame shortcut) — see docs/12.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (EqualsSubstitution, LaunchConfiguration,
                                   PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Robot spawn pose in the world = origin of the odom frame in the world.
# Must match the -x/-y handed to Gazebo below AND the planner's spawn_* params.
SPAWN_X = "-9.1"
SPAWN_Y = "-14.1"


def generate_launch_description():
    sim_pkg = FindPackageShare("vto_simulation")
    bringup_pkg = FindPackageShare("vto_bringup")
    headless = LaunchConfiguration("headless")
    controller = LaunchConfiguration("controller")   # "pursuit" or "mpc"
    sim_time = {"use_sim_time": True}

    map_yaml = PathJoinSubstitution([sim_pkg, "maps", "maze.yaml"])
    amcl_cfg = PathJoinSubstitution([bringup_pkg, "config", "amcl.yaml"])
    rviz_cfg = PathJoinSubstitution([bringup_pkg, "rviz", "maze_astar.rviz"])

    # 1) Gazebo: maze world + diffbot at the spawn cell.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([sim_pkg, "launch", "gazebo.launch.py"])),
        launch_arguments={
            "world": "maze.sdf", "x": SPAWN_X, "y": SPAWN_Y, "headless": headless,
        }.items(),
    )

    # 2) map_server: loads maze.yaml, publishes a latched /map (lifecycle node).
    map_server = Node(
        package="nav2_map_server", executable="map_server", name="map_server",
        output="screen",
        parameters=[sim_time, {"yaml_filename": map_yaml}],
    )
    # 3) lifecycle_manager: configure->activate the managed nodes in order
    #    (map_server first so amcl has a /map to localize against).
    lifecycle = Node(
        package="nav2_lifecycle_manager", executable="lifecycle_manager",
        name="lifecycle_manager_localization", output="screen",
        parameters=[sim_time, {"autostart": True,
                               "node_names": ["map_server", "amcl"]}],
    )

    # 4) AMCL: owns the map->odom transform now (REPLACES the old static transform;
    #    two publishers of map->odom would fight). Reads /scan + /map + odom TF and
    #    seeds the particle cloud at the spawn pose (see config/amcl.yaml).
    amcl = Node(
        package="nav2_amcl", executable="amcl", name="amcl", output="screen",
        parameters=[amcl_cfg],
    )

    # 5) A* global planner (map-frame; start pose from TF map->base via AMCL).
    astar = Node(
        package="vto_planning", executable="astar_planner", name="astar_planner",
        output="screen",
        parameters=[sim_time, {
            # clearance cost: R (reach) must be ~half the corridor to center the
            # path; raising w alone won't push past R. corridor 1.5 m = 15 cells.
            "clearance_weight": 6.0,   # strength
            "inflation_radius": 8,     # cells (~0.7 m ~ half corridor)
        }],
    )
    # 6) Follower (/plan -> /cmd_vel) — pick one with controller:=pursuit|mpc.
    pure_pursuit = Node(
        package="vto_control", executable="pure_pursuit", name="pure_pursuit",
        output="screen", parameters=[sim_time],
        condition=IfCondition(EqualsSubstitution(controller, "pursuit")),
    )
    mpc = Node(
        package="vto_control", executable="mpc_controller", name="mpc_controller",
        output="screen", parameters=[sim_time],
        condition=IfCondition(EqualsSubstitution(controller, "mpc")),
    )
    # 7) RViz (skipped when headless).
    rviz = Node(
        package="rviz2", executable="rviz2", name="rviz2", output="screen",
        arguments=["-d", rviz_cfg], parameters=[sim_time],
        condition=UnlessCondition(headless),
    )

    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("controller", default_value="pursuit",
                              description="follower: pursuit | mpc"),
        gazebo, map_server, lifecycle, amcl, astar, pure_pursuit, mpc, rviz,
    ])
