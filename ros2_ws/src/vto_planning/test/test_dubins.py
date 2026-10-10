"""Host-runnable tests for the Dubins curve module (no ROS needed).

Run directly:   python3 test/test_dubins.py
Or with pytest: pytest test/test_dubins.py
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from vto_planning.dubins import (  # noqa: E402
    DubinsPath, dubins_path_length, dubins_shortest_path, mod2pi, sample_dubins,
)

TOL = 1e-6


def _wrap_err(a, b):
    """Smallest absolute angular difference."""
    d = mod2pi(a - b)
    return min(d, TWO_PI - d)


TWO_PI = 2.0 * math.pi


def test_mod2pi_range():
    for t in (-10.0, -math.pi, 0.0, 0.1, math.pi, 7.0, 100.0):
        w = mod2pi(t)
        assert 0.0 <= w < TWO_PI
        assert abs(math.sin(w) - math.sin(t)) < TOL
    print("PASS test_mod2pi_range")


def test_zero_path():
    p = dubins_shortest_path((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), radius=1.0)
    assert p.length < TOL, f"identical start/goal should cost ~0, got {p.length}"
    print("PASS test_zero_path")


def test_straight_ahead_is_distance():
    # goal dead ahead, same heading -> path is just the straight segment
    p = dubins_shortest_path((0.0, 0.0, 0.0), (5.0, 0.0, 0.0), radius=1.0)
    assert abs(p.length - 5.0) < 1e-6, f"expected length 5.0, got {p.length}"
    assert p.word in ("LSL", "RSR"), f"expected a degenerate CSC word, got {p.word}"
    assert p.seg_norm[0] < TOL and p.seg_norm[2] < TOL, "turn segments should vanish"
    print(f"PASS test_straight_ahead_is_distance (word={p.word})")


def test_length_at_least_euclidean():
    cases = [
        ((0, 0, 0), (3, 2, 1.0)),
        ((1, 1, 2.0), (-4, 3, -1.5)),
        ((0, 0, 0), (0, 1, math.pi)),
        ((2, -1, 0.7), (2, -1, 3.0)),
    ]
    for s, g in cases:
        s = tuple(map(float, s))
        g = tuple(map(float, g))
        L = dubins_path_length(s, g, radius=1.3)
        euclid = math.hypot(g[0] - s[0], g[1] - s[1])
        assert L >= euclid - 1e-9, f"{s}->{g}: length {L} < euclid {euclid}"
    print("PASS test_length_at_least_euclidean")


def test_sample_endpoint_matches_goal():
    cases = [
        ((0, 0, 0), (4, 3, 1.2)),
        ((-2, 5, 2.5), (6, -3, -0.4)),
        ((0, 0, 0), (0, 0, 2.0)),          # turn-in-place-ish (CCC)
        ((1, 1, 0.3), (1.5, -2.0, 3.5)),
        ((0, 0, math.pi), (-5, 1, 0.2)),
    ]
    for s, g in cases:
        s = tuple(map(float, s))
        g = tuple(map(float, g))
        for radius in (0.8, 1.0, 2.7):
            path = dubins_shortest_path(s, g, radius)
            pts = sample_dubins(path, step=0.05)
            ex, ey, eyaw = pts[-1]
            assert math.hypot(ex - g[0], ey - g[1]) < 1e-6, \
                f"{s}->{g} r={radius} {path.word}: endpoint ({ex:.4f},{ey:.4f}) != goal"
            assert _wrap_err(eyaw, g[2]) < 1e-6, \
                f"{s}->{g} r={radius} {path.word}: end yaw {eyaw:.4f} != {g[2]}"
            # length from segments must match the polyline length of the samples
            poly = sum(math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
                       for i in range(len(pts) - 1))
            assert abs(poly - path.length) < 5e-3, \
                f"{path.word}: sampled length {poly:.4f} vs reported {path.length:.4f}"
    print("PASS test_sample_endpoint_matches_goal")


def test_curvature_never_exceeds_bound():
    radius = 1.7
    kappa_max = 1.0 / radius
    path = dubins_shortest_path((0.0, 0.0, 0.0), (3.0, 4.0, 2.0), radius)
    pts = sample_dubins(path, step=0.02)
    for i in range(len(pts) - 1):
        ds = math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
        if ds < 1e-9:
            continue
        dyaw = _wrap_err(pts[i + 1][2], pts[i][2])
        kappa = dyaw / ds
        assert kappa <= kappa_max * 1.01 + 1e-6, \
            f"segment {i}: curvature {kappa:.4f} exceeds bound {kappa_max:.4f}"
    print("PASS test_curvature_never_exceeds_bound")


def test_mirror_symmetry():
    # reflecting the whole problem across the start heading axis (y -> -y,
    # yaw -> -yaw) must give an equal-length path with L/R swapped.
    swap = str.maketrans("LR", "RL")
    for gx, gy, gyaw in [(3, 2, 0.5), (1, 4, -1.2), (-2, 1, 2.0), (5, 0.5, 3.0)]:
        a = dubins_shortest_path((0.0, 0.0, 0.0), (float(gx), float(gy), gyaw), 1.1)
        b = dubins_shortest_path((0.0, 0.0, 0.0), (float(gx), -float(gy), -gyaw), 1.1)
        assert abs(a.length - b.length) < 1e-9, \
            f"mirror length mismatch: {a.length} vs {b.length}"
        assert a.word == b.word.translate(swap), \
            f"mirror word mismatch: {a.word} vs {b.word}"
    print("PASS test_mirror_symmetry")


def test_turn_direction_matches_goal_side():
    # goal offset to the left with a left-turned heading -> first segment is L
    left = dubins_shortest_path((0.0, 0.0, 0.0), (2.0, 2.0, math.pi / 2), 1.0)
    assert left.word[0] == "L", f"expected L-first for a left goal, got {left.word}"
    right = dubins_shortest_path((0.0, 0.0, 0.0), (2.0, -2.0, -math.pi / 2), 1.0)
    assert right.word[0] == "R", f"expected R-first for a right goal, got {right.word}"
    print(f"PASS test_turn_direction_matches_goal_side ({left.word}, {right.word})")


def test_radius_scales_length():
    # a pure heading-change maneuver: doubling the radius doubles the arc length
    s, g = (0.0, 0.0, 0.0), (0.0, 0.0, 1.0)
    l1 = dubins_path_length(s, g, 1.0)
    l2 = dubins_path_length(s, g, 2.0)
    assert abs(l2 - 2.0 * l1) < 1e-6, f"{l2} != 2*{l1}"
    print("PASS test_radius_scales_length")


def test_tight_goal_uses_ccc():
    # goal close behind-ish with a big heading flip: a CCC word (RLR/LRL) should win
    p = dubins_shortest_path((0.0, 0.0, 0.0), (0.3, 0.0, math.radians(170)), 1.0)
    assert p.word in ("RLR", "LRL"), f"expected a CCC word for a tight goal, got {p.word}"
    print(f"PASS test_tight_goal_uses_ccc ({p.word})")


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for t in tests:
        t()
    print(f"\nAll {len(tests)} tests passed.")
