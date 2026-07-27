"""Host tests for the Pure Pursuit core (no ROS).

  python3 test/test_pursuit_core.py

Includes a small unicycle simulation that drives an L-shaped path and measures the
max deviation from the path with the OLD law (fixed lookahead, no pivot) vs the NEW
law (adaptive lookahead + turn-in-place + curvature slowdown).
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from vto_control.pursuit_core import (adaptive_lookahead, lookahead_point,  # noqa: E402
                                      sample_reference, velocity_command)


def test_adaptive_lookahead():
    assert adaptive_lookahead(0.0, 1.0, 0.25, 0.8) == 0.25     # stopped -> min
    assert adaptive_lookahead(0.5, 1.0, 0.25, 0.8) == 0.5      # scales with speed
    assert adaptive_lookahead(5.0, 1.0, 0.25, 0.8) == 0.8      # clamped to max
    print("PASS test_adaptive_lookahead")


def test_lookahead_ahead_on_sparse_path():
    # pruned/sparse path: closest VERTEX is behind the robot; the carrot must still
    # be interpolated AHEAD, not returned as the behind vertex (else it spins).
    path = [(0.0, 0.0), (4.0, 0.0), (4.0, 4.0)]
    for ld in (0.5, 0.8, 1.0):
        cx, cy = lookahead_point(path, 1.0, 0.0, ld)
        assert cx > 1.0, f"carrot must be ahead of x=1.0, got {(cx, cy)} at ld={ld}"
    # within ld of the end -> return the goal (last point)
    assert lookahead_point(path, 4.0, 3.7, 1.0) == (4.0, 4.0)
    print("PASS test_lookahead_ahead_on_sparse_path")


def test_sample_reference():
    # straight path along +x; window should march ahead in x at ~ds spacing, theta~0
    path = [(0.0, 0.0), (5.0, 0.0)]
    ref = sample_reference(path, x=1.0, y=0.0, n=6, ds=0.2)
    assert len(ref) == 6
    xs = [p[0] for p in ref]
    assert abs(xs[0] - 1.0) < 1e-6, "window should start at the robot's projection"
    assert all(abs(xs[k + 1] - xs[k] - 0.2) < 1e-6 for k in range(5)), "spacing = ds"
    assert all(abs(p[2]) < 1e-6 for p in ref), "tangent of a +x line is 0"
    # clamp at the end: far beyond the path -> all points park at the goal
    ref2 = sample_reference(path, x=4.9, y=0.0, n=5, ds=1.0)
    assert all(abs(p[0] - 5.0) < 1e-6 for p in ref2[1:]), "tail parks at goal"
    print("PASS test_sample_reference")


def test_turn_in_place():
    # carrot straight behind -> pivot (v == 0), and rotate toward it
    v, w, alpha = velocity_command((-1.0, 0.0), 0.0, 0.0, 0.0,
                                   max_linear=0.5, max_angular=1.5,
                                   turn_in_place_angle=0.7, turn_in_place_gain=2.0,
                                   slow_gain=1.5)
    assert v == 0.0 and abs(w) > 0.0
    print("PASS test_turn_in_place")


def test_slow_on_curvature():
    # carrot straight ahead -> near max speed; carrot off to the side -> slower
    v_straight, _, _ = velocity_command((1.0, 0.0), 0.0, 0.0, 0.0,
                                        max_linear=0.5, max_angular=1.5,
                                        turn_in_place_angle=0.7, turn_in_place_gain=2.0,
                                        slow_gain=1.5)
    v_turn, _, _ = velocity_command((1.0, 0.4), 0.0, 0.0, 0.0,
                                    max_linear=0.5, max_angular=1.5,
                                    turn_in_place_angle=0.7, turn_in_place_gain=2.0,
                                    slow_gain=1.5)
    assert v_straight > v_turn, "should slow down when the carrot curves away"
    assert v_straight <= 0.5
    print(f"PASS test_slow_on_curvature (straight={v_straight:.2f} turn={v_turn:.2f})")


# ---------------------------------------------------------------------------- #
def _l_path(step=0.1):
    """L-shaped path: (0,0)->(3,0)->(3,3), sampled every `step` m."""
    pts = [(x * step, 0.0) for x in range(int(3 / step) + 1)]
    pts += [(3.0, y * step) for y in range(1, int(3 / step) + 1)]
    return pts


def _dist_to_path(x, y, path):
    """Min distance from (x, y) to the path polyline."""
    best = float("inf")
    for (ax, ay), (bx, by) in zip(path, path[1:]):
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy or 1e-9
        t = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / L2))
        px, py = ax + t * dx, ay + t * dy
        best = min(best, math.hypot(x - px, y - py))
    return best


def _simulate(path, *, adaptive, pivot, slow, fixed_ld=0.6, dt=0.05, steps=1200):
    """Drive a unicycle along `path`; return max deviation from the path."""
    x, y, yaw, v = 0.0, 0.0, 0.0, 0.0
    gx, gy = path[-1]
    max_dev = 0.0
    for _ in range(steps):
        if math.hypot(gx - x, gy - y) < 0.2:
            break
        ld = adaptive_lookahead(v, 1.0, 0.25, 0.8) if adaptive else fixed_ld
        target = lookahead_point(path, x, y, ld) or (gx, gy)
        v, w, _ = velocity_command(
            target, x, y, yaw,
            max_linear=0.5, max_angular=1.5,
            turn_in_place_angle=(0.7 if pivot else math.pi),   # pi => never pivot
            turn_in_place_gain=2.0,
            slow_gain=(1.5 if slow else 0.0))                  # 0 => no slowdown
        x += v * math.cos(yaw) * dt
        y += v * math.sin(yaw) * dt
        yaw += w * dt
        max_dev = max(max_dev, _dist_to_path(x, y, path))
    return max_dev


def test_corner_cutting_reduced():
    path = _l_path()
    old = _simulate(path, adaptive=False, pivot=False, slow=False)   # textbook
    new = _simulate(path, adaptive=True, pivot=True, slow=True)      # upgraded
    print(f"PASS test_corner_cutting_reduced: OLD max_dev={old:.2f} m -> "
          f"NEW max_dev={new:.2f} m")
    assert new < old, "new law should cut the corner less"
    assert new < 0.20, "new law should hold the path within 0.20 m"


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for t in tests:
        t()
    print(f"\nAll {len(tests)} tests passed.")
