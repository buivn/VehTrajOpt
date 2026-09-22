"""Host-runnable tests for the Follower planner (docs/47, no ROS needed).

Run directly:   python3 test/test_follower.py
Or with pytest: pytest test/test_follower.py
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from vto_planning.astar import astar, is_free  # noqa: E402
from vto_planning.follower import (  # noqa: E402
    BicycleModel, PIDController, follow_path, plan_follower, steer_to_waypoint,
)

TOL = 1e-6
LIMITS = dict(accel_min=-0.3, accel_max=0.3, steer_vel_min=-1.0, steer_vel_max=1.0)


def _open_grid(rows=60, cols=60):
    return [[0] * cols for _ in range(rows)]


def test_pid_converges_on_constant_plant():
    """A PID commanding a plant whose value simply integrates the output
    (value += u*dt) should drive value -> desired."""
    pid = PIDController(kp=2.0, ki=0.5, kd=0.05)
    pid.set_desired(3.0)
    value = 0.0
    dt = 0.05
    for _ in range(400):
        u = pid.update(value, dt)
        value += u * dt
    assert abs(value - 3.0) < 0.05, f"expected ~3.0, got {value}"
    print("PASS test_pid_converges_on_constant_plant")


def test_bicycle_model_straight_line():
    model = BicycleModel(wheelbase=0.29, max_steer_angle=0.5, v_min=0.0, v_max=1.0)
    state = (0.0, 0.0, 0.0, 0.5, 0.0)   # x, y, theta=0, v=0.5, steer=0
    dt = 0.1
    for _ in range(10):
        state = model.step(state, (0.0, 0.0), dt)
    x, y, theta, v, steer = state
    assert abs(y) < TOL, f"straight-ahead (theta=0) should have y=0, got {y}"
    assert abs(x - 0.5) < TOL, f"expected x=0.5 (v*t=0.5*1.0), got {x}"
    print("PASS test_bicycle_model_straight_line")


def test_bicycle_model_turning_radius():
    """Constant nonzero steer angle should trace a circle of radius
    L/tan(steer) -- the textbook bicycle-model invariant, checked
    independently of how the model was derived."""
    L, steer = 0.29, 0.3
    expected_radius = L / math.tan(steer)
    model = BicycleModel(wheelbase=L, max_steer_angle=0.5, v_min=0.0, v_max=2.0)
    state = (0.0, 0.0, 0.0, 1.0, steer)
    dt = 0.001
    cx, cy = 0.0, expected_radius   # center of the turning circle, left turn
    max_err = 0.0
    for _ in range(6000):
        state = model.step(state, (0.0, 0.0), dt)
        x, y = state[0], state[1]
        r = math.hypot(x - cx, y - cy)
        max_err = max(max_err, abs(r - expected_radius))
    assert max_err < 1e-3, f"max radius deviation {max_err} too large"
    print(f"PASS test_bicycle_model_turning_radius (radius={expected_radius:.3f}, max_err={max_err:.2e})")


def test_steer_to_waypoint_sign_convention():
    """Facing +x (theta=0, steer=0), target to the left (+y) must command
    a POSITIVE steer_vel (positive steer_angle = left turn, matching
    dtheta = v*tan(steer)/L > 0 for steer > 0) -- verified against a
    worked example, not assumed from the ported C++ alone (docs/47)."""
    pid_steer, pid_vel = PIDController(), PIDController()
    state = (0.0, 0.0, 0.0, 0.3, 0.0)
    accel, steer_vel = steer_to_waypoint(state, (0.0, 1.0), pid_steer, pid_vel,
                                         cruise_speed=0.3, limits=LIMITS, dt=0.05)
    assert steer_vel > 0, f"target to the left should give steer_vel > 0, got {steer_vel}"

    pid_steer.reset(); pid_vel.reset()
    accel2, steer_vel2 = steer_to_waypoint(state, (0.0, -1.0), pid_steer, pid_vel,
                                           cruise_speed=0.3, limits=LIMITS, dt=0.05)
    assert steer_vel2 < 0, f"target to the right should give steer_vel < 0, got {steer_vel2}"
    print(f"PASS test_steer_to_waypoint_sign_convention (left={steer_vel:.3f}, right={steer_vel2:.3f})")


def test_follow_path_reaches_straight_ahead_waypoint():
    grid = _open_grid()
    model = BicycleModel(wheelbase=0.29, max_steer_angle=0.5, v_min=0.0, v_max=0.5)
    start = (30.0, 5.0, 0.0, 0.0, 0.0)   # row=30, col=5, facing +col (theta=0)
    waypoints = [(30.0, 40.0)]
    states, reached = follow_path(grid, start, waypoints, model, LIMITS,
                                  reach_tolerance=1.0, dt=0.05)
    assert reached, "should reach a clear straight-ahead waypoint"
    end = states[-1]
    assert math.hypot(end[0] - 30.0, end[1] - 40.0) <= 1.0
    print(f"PASS test_follow_path_reaches_straight_ahead_waypoint ({len(states)} states)")


def test_follow_path_stops_at_wall():
    grid = _open_grid()
    for r in range(60):
        grid[r][20] = 100   # a wall cutting straight across, column 20
    model = BicycleModel(wheelbase=0.29, max_steer_angle=0.5, v_min=0.0, v_max=0.5)
    start = (30.0, 5.0, 0.0, 0.0, 0.0)
    waypoints = [(30.0, 40.0)]   # straight through the wall
    states, reached = follow_path(grid, start, waypoints, model, LIMITS,
                                  reach_tolerance=1.0, dt=0.05)
    assert not reached, "should NOT reach through a wall"
    for (r, c, *_rest) in states:
        assert is_free(grid, int(round(r)), int(round(c)), 50), \
            "every recorded state must have been collision-free when accepted"
    print(f"PASS test_follow_path_stops_at_wall ({len(states)} states before stopping)")


def test_follow_path_respects_steer_limit():
    grid = _open_grid()
    model = BicycleModel(wheelbase=0.29, max_steer_angle=0.5, v_min=0.0, v_max=0.5)
    # a sharp zigzag of waypoints forces frequent, large steering corrections
    start = (10.0, 10.0, 0.0, 0.0, 0.0)
    waypoints = [(50.0, 15.0), (10.0, 20.0), (50.0, 25.0)]
    states, reached = follow_path(grid, start, waypoints, model, LIMITS,
                                  reach_tolerance=1.5, dt=0.05, max_steps_per_waypoint=4000)
    assert len(states) > 1
    for (_r, _c, _theta, _v, steer) in states:
        assert abs(steer) <= 0.5 + 1e-9, f"steer {steer} exceeds max_steer_angle 0.5"
    print(f"PASS test_follow_path_respects_steer_limit ({len(states)} states, reached={reached})")


def test_no_stuck_at_v0_with_large_heading_error():
    """The exact scenario that trapped the curvature-constrained MPC
    (docs/44): a huge (~180 deg) heading mismatch at a standing start.
    This module must NOT converge to (accel=0, steer_vel=0) -- both PIDs
    are independent, so accel should be nonzero from the very first tick
    regardless of the heading error (docs/47's central claim, checked
    directly rather than just argued)."""
    grid = _open_grid()
    model = BicycleModel(wheelbase=0.29, max_steer_angle=0.5, v_min=0.0, v_max=0.5)
    start = (30.0, 30.0, math.pi / 2, 0.0, 0.0)   # facing "north" (+row... )
    waypoint = (30.0, 30.0 - 1e-6)                # effectively due "south"-ish/behind
    pid_steer, pid_vel = PIDController(), PIDController()
    xy_state = (start[1], start[0], start[2], start[3], start[4])
    accel, steer_vel = steer_to_waypoint(xy_state, (waypoint[1], waypoint[0]),
                                         pid_steer, pid_vel, cruise_speed=0.5,
                                         limits=LIMITS, dt=0.05)
    assert accel > 0.0, f"velocity PID must push forward regardless of heading error, got accel={accel}"
    assert abs(steer_vel) == LIMITS["steer_vel_max"] or abs(steer_vel) > 0.5, \
        f"steering PID should saturate hard on a near-180 deg error, got steer_vel={steer_vel}"
    print(f"PASS test_no_stuck_at_v0_with_large_heading_error (accel={accel:.3f}, steer_vel={steer_vel:.3f})")


def test_plan_follower_end_to_end_with_obstacle():
    grid = _open_grid(60, 60)
    for r in range(0, 35):
        grid[r][30] = 100
    for r in range(45, 60):
        grid[r][30] = 100
    # a gap in the wall between rows 35-45 near column 30
    start = (10.0, 10.0, 0.0, 0.0, 0.0)
    goal = (40, 50)
    astar_fn = lambda g, s, gl: astar(g, (int(s[0]), int(s[1])), gl)  # noqa: E731
    states, reached = plan_follower(grid, start, goal, astar_fn,
                                    wheelbase=0.29, max_steer_angle=0.5,
                                    v_max=0.5, reach_tolerance=1.5, dt=0.05,
                                    max_steps_per_waypoint=3000)
    assert states is not None, "A* itself should find a path through the gap"
    assert reached, "follower should reach the goal through the gap"
    end = states[-1]
    assert math.hypot(end[0] - goal[0], end[1] - goal[1]) <= 1.5
    print(f"PASS test_plan_follower_end_to_end_with_obstacle ({len(states)} states)")


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for t in tests:
        t()
    print(f"\nAll {len(tests)} tests passed.")
