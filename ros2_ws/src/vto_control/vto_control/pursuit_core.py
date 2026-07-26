"""Pure Pursuit control law — NO ROS dependencies.

Framework-free (like vto_planning/astar.py) so it is unit-testable on the host and
so the ROS node stays a thin adapter. The node feeds pose + path + measured speed;
these functions return the carrot and the (v, w) command.

Three robustness features over textbook fixed-lookahead pursuit (see docs/10):
  1. adaptive_lookahead: Ld grows with speed -> smooth on straights, tight in turns.
  2. turn-in-place: when the carrot is far off-heading, stop and pivot (diff-drive
     can rotate in place) instead of arcing through the corner (and into a wall).
  3. curvature slowdown: forward speed drops as the commanded turn sharpens.
"""
import math


def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


def closest_index(path, x, y):
    """Index of the path point nearest (x, y) — tracks progress along the path."""
    best_i, best_d = 0, float("inf")
    for i, (px, py) in enumerate(path):
        d = (px - x) ** 2 + (py - y) ** 2
        if d < best_d:
            best_d, best_i = d, i
    return best_i


def lookahead_point(path, x, y, ld):
    """Carrot at arc-length `ld` ahead of the robot's PROJECTION onto the path.

    Walks forward segment-by-segment and interpolates, so the carrot is always
    ahead even when the path is SPARSE (e.g. pruned to endpoints on a straight) —
    a vertex-only search can otherwise return the closest vertex, which may sit
    *behind* the robot, making it spin. Returns the last point (goal) when the
    path ends within `ld`; None only for an empty path.
    """
    if not path:
        return None
    if len(path) == 1:
        return path[0]

    # 1) closest projection onto the polyline -> (segment i, param t in [0,1])
    best_d2, best_i, best_t = float("inf"), 0, 0.0
    for i in range(len(path) - 1):
        ax, ay = path[i]
        dx, dy = path[i + 1][0] - ax, path[i + 1][1] - ay
        seg2 = dx * dx + dy * dy or 1e-12
        t = clamp(((x - ax) * dx + (y - ay) * dy) / seg2, 0.0, 1.0)
        cx, cy = ax + t * dx, ay + t * dy
        d2 = (x - cx) ** 2 + (y - cy) ** 2
        if d2 < best_d2:
            best_d2, best_i, best_t = d2, i, t

    # 2) walk FORWARD from the projection, accumulating length until we cover ld
    remaining, i, t = ld, best_i, best_t
    while i < len(path) - 1:
        ax, ay = path[i]
        bx, by = path[i + 1]
        seg_len = math.hypot(bx - ax, by - ay) or 1e-12
        left = (1.0 - t) * seg_len              # distance to this segment's end
        if remaining <= left:
            f = t + remaining / seg_len
            return (ax + (bx - ax) * f, ay + (by - ay) * f)
        remaining -= left
        i, t = i + 1, 0.0
    return path[-1]                             # ran off the end -> aim at the goal


def adaptive_lookahead(v_meas, gain, ld_min, ld_max):
    """Ld = clamp(gain * |v|, ld_min, ld_max). Fast -> long (stable on straights);
    slow/stopped -> short (tight tracking through corners)."""
    return clamp(gain * abs(v_meas), ld_min, ld_max)


def sample_reference(path, x, y, n, ds):
    """Turn a global path into an MPC reference window of `n` states.

    Returns `n` points [(x, y, theta), …] sampled ALONG the path starting at the
    robot's projection onto it, spaced by arc-length `ds`, clamped at the path end
    (so the tail "parks" at the goal). `theta` is the local path tangent. Pure math
    (no numpy/ROS) so it is host-testable; the MPC node stacks these into (3, n).
    """
    if not path:
        return []
    if len(path) == 1:
        px, py = path[0]
        return [(px, py, 0.0)] * n

    # per-segment geometry + cumulative arc length at each vertex
    segs, cum = [], [0.0]
    for (ax, ay), (bx, by) in zip(path, path[1:]):
        L = math.hypot(bx - ax, by - ay)
        segs.append((ax, ay, bx, by, L, math.atan2(by - ay, bx - ax)))
        cum.append(cum[-1] + L)
    total = cum[-1]

    # project the robot onto the path -> starting arc length s0
    best_d2, s0 = float("inf"), 0.0
    for i, (ax, ay, bx, by, L, _) in enumerate(segs):
        dx, dy = bx - ax, by - ay
        t = 0.0 if L == 0 else clamp(((x - ax) * dx + (y - ay) * dy) / (L * L), 0.0, 1.0)
        cx, cy = ax + t * dx, ay + t * dy
        d2 = (x - cx) ** 2 + (y - cy) ** 2
        if d2 < best_d2:
            best_d2, s0 = d2, cum[i] + t * L

    def at(s):
        s = clamp(s, 0.0, total)
        for i in range(len(segs)):
            if s <= cum[i + 1] or i == len(segs) - 1:
                ax, ay, bx, by, L, th = segs[i]
                t = 0.0 if L == 0 else (s - cum[i]) / L
                return (ax + t * (bx - ax), ay + t * (by - ay), th)
        return None  # unreachable

    return [at(s0 + k * ds) for k in range(n)]


def velocity_command(target, px, py, yaw, *, max_linear, max_angular,
                     turn_in_place_angle, turn_in_place_gain, slow_gain,
                     min_linear=0.12):
    """Compute (v, w) to chase `target` from pose (px, py, yaw).

    Returns (v, w, alpha) where alpha is the heading error to the carrot (rad).

    `min_linear` is an anti-stall floor: whenever we're NOT pivoting, drive at
    least this fast so the robot physically moves (overcomes wheel deadband) and
    builds the speed that grows the adaptive lookahead — otherwise a small carrot
    + curvature slowdown can collapse v to ~0 and the robot only spins.
    """
    # transform the target into the robot frame (x forward, y left)
    dx, dy = target[0] - px, target[1] - py
    x_r = math.cos(-yaw) * dx - math.sin(-yaw) * dy
    y_r = math.sin(-yaw) * dx + math.cos(-yaw) * dy
    alpha = math.atan2(y_r, x_r)                 # angle to carrot

    # (2) turn-in-place: carrot too far off-heading -> pivot, don't translate.
    if abs(alpha) > turn_in_place_angle:
        w = clamp(turn_in_place_gain * alpha, -max_angular, max_angular)
        return 0.0, w, alpha

    # pure-pursuit curvature: kappa = 2 * y_r / dist^2  ( = 2 sin(alpha)/dist )
    dist = max(math.hypot(x_r, y_r), 1e-6)
    curvature = 2.0 * y_r / (dist * dist)

    # (3) slow on curvature, but never below the anti-stall floor.
    v = max(min_linear, max_linear / (1.0 + slow_gain * abs(curvature)))
    w = clamp(v * curvature, -max_angular, max_angular)
    return v, w, alpha
