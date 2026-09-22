"""Host-runnable tests for the Ackermann control-set + search (no ROS needed).

Run directly:   python3 test/test_lattice_ackermann.py
Or with pytest: pytest test/test_lattice_ackermann.py
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from vto_planning.dubins import mod2pi  # noqa: E402
from vto_planning.lattice import (  # noqa: E402
    N_HEADINGS, densify_path, heading_to_index, index_to_heading,
)
from vto_planning.lattice_ackermann import (  # noqa: E402
    ackermann_primitive_table, make_ackermann_primitive_library, plan_ackermann,
)

FREE, OCC = 0, 100
R_CELLS = 3.0
REACH = 5

# build the control set once and reuse across tests
LIB = make_ackermann_primitive_library(turning_radius_cells=R_CELLS,
                                       max_reach_cells=REACH)
TABLE = ackermann_primitive_table(turning_radius_cells=R_CELLS,
                                  max_reach_cells=REACH)


def grid_from(rows):
    return [[OCC if ch == "#" else FREE for ch in row] for row in rows]


def _endpoint(prim):
    x, y, _ = prim["samples"][-1]
    return int(round(x)), int(round(y))


def test_library_nonempty_and_has_straight():
    assert len(LIB) >= 4, f"control set too small: {len(LIB)}"
    straights = [p for p in LIB if p["dh"] == 0]
    assert straights, "expected a straight-ahead primitive"
    er, ec = _endpoint(straights[0])
    assert ec == 0 and er > 0, f"straight primitive should advance along +row, got ({er},{ec})"
    print(f"PASS test_library_nonempty_and_has_straight (|control set|={len(LIB)})")


def test_branching_factor_is_small():
    # the whole point of pruning: keep A*'s branching factor down
    assert 4 <= len(LIB) <= 32, f"unexpected control-set size {len(LIB)}"
    print(f"PASS test_branching_factor_is_small (|control set|={len(LIB)})")


def test_every_primitive_lands_on_a_heading_bin():
    for p in LIB:
        _, _, end_yaw = p["samples"][-1]
        assert heading_to_index(end_yaw) == p["dh"] % N_HEADINGS, \
            f"primitive end yaw {end_yaw:.4f} != bin {p['dh']}"
    print("PASS test_every_primitive_lands_on_a_heading_bin")


def test_every_primitive_respects_turning_radius():
    kappa_max = 1.0 / R_CELLS
    for p in LIB:
        s = p["samples"]
        for i in range(len(s) - 1):
            ds = math.hypot(s[i + 1][0] - s[i][0], s[i + 1][1] - s[i][1])
            if ds < 1e-9:
                continue
            dyaw = mod2pi(s[i + 1][2] - s[i][2])
            dyaw = min(dyaw, 2 * math.pi - dyaw)
            assert dyaw / ds <= kappa_max * 1.05 + 1e-6, \
                f"primitive dh={p['dh']} curvature {dyaw / ds:.4f} > {kappa_max:.4f}"
    print("PASS test_every_primitive_respects_turning_radius")


def test_pruning_drops_composable_moves():
    ends = {(_endpoint(p) + (p["dh"] % N_HEADINGS,)) for p in LIB}
    # a straight 1-cell move exists; a straight 2-cell move is two of those -> pruned
    assert (1, 0, 0) in ends, "unit straight primitive missing"
    assert (2, 0, 0) not in ends, "2-cell straight should be pruned (composable)"
    assert (3, 0, 0) not in ends, "3-cell straight should be pruned (composable)"
    print("PASS test_pruning_drops_composable_moves")


def test_table_has_expected_shape():
    assert len(TABLE) == N_HEADINGS
    for prims in TABLE:
        for p in prims:
            assert {"dh", "cost", "samples", "dr", "dc"} <= set(p)
            assert p["cost"] > 0.0
    print("PASS test_table_has_expected_shape")


def test_plan_down_open_corridor():
    g = grid_from(["." * 24 for _ in range(5)])
    # heading index 4 points along +col in this convention
    path = plan_ackermann(g, (2, 0, 4), (2, 22), primitive_table=TABLE,
                          goal_tolerance=1.5)
    assert path is not None
    assert (path[0][0], path[0][1]) == (2, 0)
    dense = densify_path(path, TABLE)
    for (r, c) in dense:
        assert g[int(round(r))][int(round(c))] == FREE
    print(f"PASS test_plan_down_open_corridor (states={len(path)})")


def test_plan_around_wall_is_feasible():
    g = grid_from([
        "..................",
        "..................",
        "..................",
        "..................",
        "........#########.",
        "..................",
        "..................",
        "..................",
        "..................",
    ])
    path = plan_ackermann(g, (2, 1, 4), (6, 1), primitive_table=TABLE,
                          goal_tolerance=2.0)
    assert path is not None, "should route around the wall"
    dense = densify_path(path, TABLE)
    bad = sum(1 for (r, c) in dense
              if g[int(round(r))][int(round(c))] != FREE)
    assert bad == 0, f"{bad} dense path points hit an obstacle"
    # a car cannot pivot: every step must actually translate
    for a, b in zip(path, path[1:]):
        assert (a[0], a[1]) != (b[0], b[1]), "consecutive states share a cell (pivot!)"
    print(f"PASS test_plan_around_wall_is_feasible (states={len(path)}, pts={len(dense)})")


def test_walled_off_goal_returns_none():
    g = grid_from([
        "......#......",
        "......#......",
        "......#......",
        "......#......",
        "......#......",
    ])
    path = plan_ackermann(g, (2, 0, 4), (2, 11), primitive_table=TABLE,
                          goal_tolerance=0.6)
    assert path is None
    print("PASS test_walled_off_goal_returns_none")


def test_plan_path_curvature_typically_bounded():
    # Menger curvature over the dense path. Sub-cell drift at primitive joins
    # (docs/25 "sub-cell drift" caveat) produces isolated spikes, so we check
    # the MEDIAN, which is robust to ~1 outlier per join: the body of the path
    # must not exceed the turning-radius bound by much.
    g = grid_from(["." * 30 for _ in range(12)])
    path = plan_ackermann(g, (6, 1, 4), (2, 26), primitive_table=TABLE,
                          goal_tolerance=1.5)
    assert path is not None
    d = densify_path(path, TABLE)
    kappa_max = 1.0 / R_CELLS
    kappas = []
    for i in range(1, len(d) - 1):
        a, b, c = d[i - 1], d[i], d[i + 1]
        ab = math.hypot(b[0] - a[0], b[1] - a[1])
        bc = math.hypot(c[0] - b[0], c[1] - b[1])
        ca = math.hypot(a[0] - c[0], a[1] - c[1])
        if ab < 1e-6 or bc < 1e-6 or ca < 1e-6:
            continue
        area = abs((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])) / 2.0
        kappas.append(4.0 * area / (ab * bc * ca))
    kappas.sort()
    median = kappas[len(kappas) // 2]
    assert median <= kappa_max * 1.35, \
        f"median path curvature {median:.3f} exceeds bound {kappa_max:.3f}"
    print(f"PASS test_plan_path_curvature_typically_bounded "
          f"(median={median:.3f}, max_allowed={kappa_max:.3f})")


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for t in tests:
        t()
    print(f"\nAll {len(tests)} tests passed.")
