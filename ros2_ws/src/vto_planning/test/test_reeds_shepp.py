"""Host-runnable tests for the Reeds-Shepp curve module (no ROS needed).

Correctness matters more here than in most modules in this repo (docs/41:
the closed-form CSC formulas and the numeric CCC solve were hand-derived,
not transcribed from a trusted reference) -- these tests are the actual
verification, not a formality. test_sample_endpoint_matches_goal is the
load-bearing one: it doesn't trust any word/segment derivation, only that
forward-simulating whatever came out actually lands on the goal.

Run directly:   python3 test/test_reeds_shepp.py
Or with pytest: pytest test/test_reeds_shepp.py
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from vto_planning.dubins import dubins_path_length, mod2pi  # noqa: E402
from vto_planning.reeds_shepp import (  # noqa: E402
    reeds_shepp_path_length, reeds_shepp_shortest_path, sample_reeds_shepp,
)

TOL = 1e-6
TWO_PI = 2.0 * math.pi


def _wrap_err(a, b):
    d = mod2pi(a - b)
    return min(d, TWO_PI - d)


def test_zero_path():
    p = reeds_shepp_shortest_path((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), radius=1.0)
    assert p.length < TOL, f"identical start/goal should cost ~0, got {p.length}"
    print("PASS test_zero_path")


def test_straight_ahead_is_distance():
    p = reeds_shepp_shortest_path((0.0, 0.0, 0.0), (5.0, 0.0, 0.0), radius=1.0)
    assert abs(p.length - 5.0) < 1e-6, f"expected length 5.0, got {p.length}"
    print("PASS test_straight_ahead_is_distance")


def test_sample_endpoint_matches_goal():
    """The load-bearing test: for many configurations, forward-simulating
    the returned segments must land exactly on the goal, regardless of
    which word (closed-form CSC, numeric CCC, or plain Dubins) was used."""
    cases = [
        ((0, 0, 0), (4, 3, 1.2)),
        ((-2, 5, 2.5), (6, -3, -0.4)),
        ((0, 0, 0), (0, 0, 2.0)),
        ((1, 1, 0.3), (1.5, -2.0, 3.5)),
        ((0, 0, math.pi), (-5, 1, 0.2)),
        ((0, 0, 0), (0.3, 0.0, math.radians(170))),   # tight, near-behind
        ((0, 0, 0), (0.0, 0.0, math.pi)),             # pure U-turn, same spot
        ((0, 0, 0), (-2.0, 0.3, math.radians(160))),  # behind-ish, needs reverse
        ((3, -1, 1.0), (3.2, -0.9, 1.0 + math.pi)),   # small offset, near-opposite heading
    ]
    for s, g in cases:
        s = tuple(map(float, s))
        g = tuple(map(float, g))
        for radius in (0.53, 1.0, 2.7):
            path = reeds_shepp_shortest_path(s, g, radius)
            pts = sample_reeds_shepp(path, step=0.05)
            ex, ey, eyaw = pts[-1]
            assert math.hypot(ex - g[0], ey - g[1]) < 1e-5, \
                f"{s}->{g} r={radius} {path.segments}: endpoint ({ex:.4f},{ey:.4f}) != goal {g[:2]}"
            assert _wrap_err(eyaw, g[2]) < 1e-5, \
                f"{s}->{g} r={radius} {path.segments}: end yaw {eyaw:.4f} != {g[2]}"
            poly = sum(math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
                       for i in range(len(pts) - 1))
            assert abs(poly - path.length) < 5e-3, \
                f"{path.segments}: sampled length {poly:.4f} vs reported {path.length:.4f}"
    print("PASS test_sample_endpoint_matches_goal")


def test_never_worse_than_dubins():
    """Reverse is an additional option, never a requirement -- Reeds-Shepp
    length must never exceed the forward-only Dubins length for the same
    pose pair."""
    cases = [
        ((0, 0, 0), (5, 2, 1.0)),
        ((0, 0, 0), (10, 0, 0.0)),
        ((1, -1, 0.5), (8, 4, -1.5)),
        ((0, 0, 0), (2, 0, math.pi / 2)),
    ]
    for s, g in cases:
        s, g = tuple(map(float, s)), tuple(map(float, g))
        for radius in (0.53, 1.5):
            rs = reeds_shepp_path_length(s, g, radius)
            db = dubins_path_length(s, g, radius)
            assert rs <= db + 1e-6, f"{s}->{g} r={radius}: RS {rs:.4f} > Dubins {db:.4f}"
    print("PASS test_never_worse_than_dubins")


def test_reverse_beats_dubins_on_u_turn():
    """The whole point: a pure U-turn in place (same position, opposite
    heading) is expensive for a forward-only car (a wide loop) but cheap
    with reverse (a short forward-back V). Reeds-Shepp must be
    substantially shorter here, not just tied."""
    s, g, radius = (0.0, 0.0, 0.0), (0.0, 0.0, math.pi), 1.0
    rs = reeds_shepp_path_length(s, g, radius)
    db = dubins_path_length(s, g, radius)
    assert rs < 0.5 * db, f"expected RS << Dubins for a U-turn, got RS={rs:.3f} Dubins={db:.3f}"
    print(f"PASS test_reverse_beats_dubins_on_u_turn (RS={rs:.3f}, Dubins={db:.3f})")


def test_curvature_never_exceeds_bound():
    radius = 1.7
    kappa_max = 1.0 / radius
    path = reeds_shepp_shortest_path((0.0, 0.0, 0.0), (0.5, 0.2, math.radians(160)), radius)
    pts = sample_reeds_shepp(path, step=0.02)
    for i in range(len(pts) - 1):
        ds = math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
        if ds < 1e-9:
            continue
        dyaw = _wrap_err(pts[i + 1][2], pts[i][2])
        kappa = dyaw / ds
        assert kappa <= kappa_max * 1.01 + 1e-6, \
            f"segment {i}: curvature {kappa:.4f} exceeds bound {kappa_max:.4f}"
    print("PASS test_curvature_never_exceeds_bound")


def test_radius_scales_length():
    s, g = (0.0, 0.0, 0.0), (0.0, 0.0, math.pi)
    l1 = reeds_shepp_path_length(s, g, 1.0)
    l2 = reeds_shepp_path_length(s, g, 2.0)
    assert abs(l2 - 2.0 * l1) < 1e-4, f"{l2} != 2*{l1}"
    print("PASS test_radius_scales_length")


def test_solves_docs40_heading9_trap():
    """The actual motivating case (docs/40): from a specific heading in an
    open room, forward-only search exhausted 877 reachable states without
    escaping to ANY of 15 candidate goals 10-25m away. Reeds-Shepp must
    solve at least one of them from that exact heading."""
    # local coordinates: heading 9 = 202.5 deg in lattice.py's convention
    # (index * 2*pi/16); reproduced here directly in (x,y,yaw) terms Rather
    # than depending on lattice.py's row/col framing.
    yaw9 = 9 * (2.0 * math.pi / 16.0)
    start = (0.0, 0.0, yaw9)
    # a handful of directions/distances standing in for "some goal 10-25
    # cells away, in the direction the room's exits actually are" (docs/40:
    # heading 9 could reach nothing; headings ~270-90 deg could).
    targets = [(15.0, 3.0, 0.0), (-12.0, -4.0, math.pi / 2), (18.0, -6.0, -0.3)]
    radius = 5.3   # ~ R_min in cells at this map's resolution (docs/39)
    solved = 0
    for gx, gy, gyaw in targets:
        try:
            reeds_shepp_shortest_path(start, (gx, gy, gyaw), radius)
            solved += 1
        except RuntimeError:
            pass
    assert solved > 0, "Reeds-Shepp should solve at least one target from the trapped heading"
    print(f"PASS test_solves_docs40_heading9_trap ({solved}/{len(targets)} solved)")


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for t in tests:
        t()
    print(f"\nAll {len(tests)} tests passed.")
