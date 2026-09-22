"""Follower planner (docs/47): A* guidance + PID-driven dynamic-bicycle-model
trajectory generation for the Ackermann robot. NO ROS dependencies.

Ported from the advisor's src/GP (Copyright (C) 2023 Erion Plaku, reused
here with verbal permission for new projects, per CLAUDE.md) -- specifically
Utils/PIDController.hpp and MPSnakeSimulator.cpp's MotionEqs/SteerToPosition
(the tractor-unit dynamics; there is no separate plain-car class in src/GP,
the car model lives inside the snake simulator and the snake's trailer
links are simply not used here).

Two deliberate departures from the ported C++, both to match THIS robot
rather than the advisor's generic research one:
  - Rear-wheel-driving bicycle equations (dtheta = v*tan(steer)/L), not
    MPSnakeSimulator's front-wheel-driving default (dtheta = v*sin(steer)/L)
    -- ackermannbot.urdf.xacro drives its REAR wheels and steers the FRONT
    ones (docs/31), the standard convention pure_pursuit_ackermann's own
    max_curvature = tan(max_steer_angle)/L already uses.
  - Physical limits (wheelbase, max_steer_angle, max_vel) come from THIS
    project's own ackermannbot constants, not GPConstants.hpp's ROBOT_*
    defaults (1.75 m body, +-88 deg steering) -- those were already flagged
    in docs/31 as a different, larger scenario robot's unconstrained-
    testing defaults, not a fit here.

Scope (docs/47): this is the DETERMINISTIC reading of "Follower" -- A*
gives a waypoint sequence, this module's PID+model chases each waypoint in
turn, collision-checked at every simulated step. The advisor's actual
FollowPlanner wraps this same steering law in a randomized RRT (a follow
corridor biasing tree exploration, RRT/EST/GUST fallback modes) for
robustness when a straight chase would get stuck -- deliberately NOT
ported here: a much larger undertaking than what's needed, and this
module's own collision check already turns "would have needed the RRT's
robustness" into an honest, visible failure (stops and reports) rather
than silently doing something unsafe.

Why this doesn't hit the MPC's "stuck at v=0" trap (docs/44): steer angle
is its own STATE here, independent of velocity -- the steering PID can
"pre-aim" the front wheel while v=0 (a real steering wheel turns whether
or not the car is moving), and the velocity PID never looks at heading
error at all, so nothing here can rationally converge to commanding
(accel=0, steer_vel=0) indefinitely the way a curvature-COUPLED optimizer
can. It can still fail by COLLISION if a reorientation needs more room
than the map has -- a different failure mode, not immunity.
"""
import math

from vto_planning.astar import is_free

TWO_PI = 2.0 * math.pi


def _wrap_pi(theta):
    """Wrap an angle to (-pi, pi] -- signed, since PID error terms need a sign."""
    return theta - TWO_PI * math.floor((theta + math.pi) / TWO_PI)


class PIDController:
    """Direct port of Utils/PIDController.hpp -- textbook PID, unchanged
    (Kp*e + Ki*integral(e) + Kd*de/dt). Defaults (10, 0.1, 0.1) are the
    advisor's own, tuned for a 1.75 m / 88 deg-steering robot -- kept as
    STARTING points here, not assumed correct at this project's much
    smaller scale (0.29 m wheelbase, 0.5 rad steering limit); the tests in
    test_follower.py check convergence, not tuning quality specifically."""

    def __init__(self, kp=10.0, ki=0.1, kd=0.1):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.desired = 0.0
        self._integral = 0.0
        self._prev_error = 0.0

    def set_desired(self, value):
        self.desired = value

    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0

    def update(self, current, dt):
        error = self.desired - current
        self._integral += error * dt
        u = (self.kp * error + self.ki * self._integral
            + self.kd * (error - self._prev_error) / dt)
        self._prev_error = error
        return u


class BicycleModel:
    """Dynamic bicycle model -- state (x, y, theta, v, steer), control
    (accel, steer_vel). Ported from MPSnakeSimulator::MotionEqs (rear-
    wheel-driving branch, see module docstring) and SimulateOneStep
    (explicit Euler, matching the original exactly -- no reason to use a
    fancier integrator it didn't use).

    Steering angle and velocity are STATES, not instantaneous commands: a
    real steering wheel can be turned before the car ever moves, and a
    real car can't snap to a new speed.

    v and steer_angle are clamped to their physical limits after each
    step -- a deliberate departure from the original, which instead
    REJECTS an out-of-bound state as invalid (fine for its RRT, which can
    just try a different random branch; too brittle for this module's
    single deterministic pass, where a marginal one-step overshoot would
    otherwise kill the whole trajectory for no real benefit)."""

    def __init__(self, wheelbase, max_steer_angle, v_min, v_max):
        self.L = wheelbase
        self.max_steer_angle = max_steer_angle
        self.v_min, self.v_max = v_min, v_max

    def step(self, state, control, dt):
        """control = (accel, steer_vel), already clamped by the caller
        (steer_to_waypoint) to actuator rate limits -- this method only
        clamps the resulting STATE (v, steer) to physical position/speed
        limits, a different thing."""
        x, y, theta, v, steer = state
        accel, steer_vel = control
        dtheta = v * math.tan(steer) / self.L
        nx = x + v * math.cos(theta) * dt
        ny = y + v * math.sin(theta) * dt
        ntheta = _wrap_pi(theta + dtheta * dt)
        nv = min(self.v_max, max(self.v_min, v + accel * dt))
        nsteer = min(self.max_steer_angle, max(-self.max_steer_angle, steer + steer_vel * dt))
        return (nx, ny, ntheta, nv, nsteer)


def steer_to_waypoint(state, target, pid_steer, pid_vel, cruise_speed, limits, dt):
    """Port of MPSnakeSimulator::SteerToPosition -- two INDEPENDENT PID
    loops (the velocity one never looks at heading error, the steering one
    never looks at velocity; see module docstring for why that matters).

    heading_error = signed angle FROM the current heading+steer direction
    TO the target direction (positive = target is to the left, matching
    positive steer_angle = left turn in dtheta = v*tan(steer)/L). This is
    the negation of the original's VecFromToAngleCCW(u, v) (angle from
    target-direction TO current-facing) -- verified by working a concrete
    example (facing +x, target at +y => heading_error=+90 deg, correctly
    commanding a positive/left steer_vel) rather than assumed from the
    C++ alone.

    Returns (accel, steer_vel), clamped to (accel_min/max, steer_vel_min/max).
    """
    x, y, theta, v, steer = state
    tx, ty = target
    to_target = math.atan2(ty - y, tx - x)
    facing = theta + steer
    heading_error = _wrap_pi(to_target - facing)

    pid_steer.set_desired(0.0)
    steer_vel = pid_steer.update(-heading_error, dt)
    steer_vel = min(limits["steer_vel_max"], max(limits["steer_vel_min"], steer_vel))

    pid_vel.set_desired(cruise_speed)
    accel = pid_vel.update(v, dt)
    accel = min(limits["accel_max"], max(limits["accel_min"], accel))

    return (accel, steer_vel)


def follow_path(grid, start_state, waypoints, model, limits, cruise_speed=None,
                reach_tolerance=1.0, dt=0.05, max_steps_per_waypoint=2000,
                occ_thresh=50, kp_steer=10.0, ki_steer=0.1, kd_steer=0.1,
                kp_vel=10.0, ki_vel=0.1, kd_vel=0.1):
    """Layer 2: chase `waypoints` (list of (row, col), cell units) in
    sequence from `start_state` (row, col, theta, v, steer, cell units for
    position), simulating the bicycle model forward, collision-checking
    (`is_free`, same inflated grid + point-check convention as astar.py /
    lattice.py) at every step.

    Returns (states, reached) -- states is the dense list of simulated
    (row, col, theta, v, steer) tuples up to and including the failure or
    final-waypoint-reached point; reached is False if collision or
    max_steps_per_waypoint was hit on some waypoint before finishing.
    cruise_speed defaults to model.v_max (drive as fast as the model
    allows, matching a reasonable reading of "cruise" absent the
    original's randomized RRT-diversity speed).
    """
    if cruise_speed is None:
        cruise_speed = model.v_max
    pid_steer = PIDController(kp_steer, ki_steer, kd_steer)
    pid_vel = PIDController(kp_vel, ki_vel, kd_vel)

    state = tuple(start_state)
    states = [state]
    for wp in waypoints:
        pid_steer.reset()
        pid_vel.reset()
        reached_wp = False
        for _ in range(max_steps_per_waypoint):
            r, c = state[0], state[1]
            if math.hypot(wp[0] - r, wp[1] - c) <= reach_tolerance:
                reached_wp = True
                break
            xy_state = (state[1], state[0], state[2], state[3], state[4])  # (x,y,...)=(col,row,...)
            control = steer_to_waypoint(
                xy_state, (wp[1], wp[0]), pid_steer, pid_vel, cruise_speed, limits, dt)
            x, y, theta, v, steer = model.step(xy_state, control, dt)
            new_state = (y, x, theta, v, steer)   # back to (row, col, ...)
            # is_free needs integer cell indices; position stays continuous
            # in the returned trajectory (rounding is only for the grid lookup)
            if not is_free(grid, int(round(new_state[0])), int(round(new_state[1])), occ_thresh):
                return states, False
            state = new_state
            states.append(state)
        if not reached_wp:
            return states, False
    return states, True


def plan_follower(grid, start, goal, astar_fn, wheelbase, max_steer_angle,
                  v_min=0.0, v_max=0.5, accel_min=-0.3, accel_max=0.3,
                  steer_vel_min=-1.0, steer_vel_max=1.0, **kwargs):
    """Top-level: layer 1 (A*, via `astar_fn` -- pass vto_planning.astar.astar
    or a partial of it with connectivity/clearance kwargs already bound) ->
    layer 2 (follow_path). `start` is (row, col, theta, v, steer); `goal`
    is (row, col). Returns (states, reached) or (None, False) if A* itself
    finds no path.

    accel/steer_vel limits (+-0.3 m/s^2, +-1.0 rad/s) are NOT ported from
    GPConstants.hpp (+-2, +-3 -- tuned for the advisor's 1.75 m robot) --
    picked fresh for ackermannbot's scale, same spirit as docs/39's
    turning-radius derivation: conservative enough to be plausible for a
    small robot, explicitly flagged as picked-not-measured, same as
    docs/31 already did for this robot's steering angle.
    """
    path_cells = astar_fn(grid, (start[0], start[1]), goal)
    if path_cells is None:
        return None, False
    model = BicycleModel(wheelbase, max_steer_angle, v_min, v_max)
    limits = dict(accel_min=accel_min, accel_max=accel_max,
                 steer_vel_min=steer_vel_min, steer_vel_max=steer_vel_max)
    return follow_path(grid, start, path_cells[1:], model, limits, **kwargs)
