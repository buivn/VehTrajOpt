"""Host-runnable tests for the pure A* module (no ROS needed).

Run directly:   python3 test/test_astar.py
Or with pytest: pytest test/test_astar.py
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from vto_planning.astar import astar, clearance_field, prune_collinear  # noqa: E402

# 0 = free, 100 = occupied. Row 0 is the top.
FREE, OCC = 0, 100


def grid_from(rows):
    """Build a grid from an ASCII map: '#' occupied, '.' free."""
    return [[OCC if ch == "#" else FREE for ch in row] for row in rows]


def test_straight_line():
    g = grid_from(["....."])
    path = astar(g, (0, 0), (0, 4), connectivity=4)
    assert path is not None
    assert path[0] == (0, 0) and path[-1] == (0, 4)
    assert len(path) == 5
    print("PASS test_straight_line")


def test_goes_around_wall():
    #  S . #
    #  # . #
    #  # . G   -> must snake through the middle column
    g = grid_from([
        "..#",
        "#.#",
        "#.#",
    ])
    path = astar(g, (0, 0), (2, 1), connectivity=8)
    assert path is not None, "should find a way around the wall"
    for (r, c) in path:
        assert g[r][c] == FREE, "path must never enter an occupied cell"
    print(f"PASS test_goes_around_wall (len={len(path)})")


def test_unreachable_returns_none():
    #  goal is walled off completely
    g = grid_from([
        "..#.",
        "..#.",
        "..#.",
    ])
    path = astar(g, (0, 0), (0, 3), connectivity=8)
    assert path is None, "walled-off goal must return None"
    print("PASS test_unreachable_returns_none")


def test_no_corner_cutting():
    #  A diagonal squeeze between two walls must NOT be cut through.
    #  . #
    #  # .   -> (0,0) to (1,1) diagonally is blocked by both walls
    g = grid_from([
        ".#",
        "#.",
    ])
    path = astar(g, (0, 0), (1, 1), connectivity=8)
    assert path is None, "must not cut the corner between two diagonal walls"
    print("PASS test_no_corner_cutting")


def test_clearance_centers_path():
    # A 3-wide corridor (rows 0..2 free, walls above/below via borders).
    # Without clearance A* hugs a wall row; with clearance it prefers the middle.
    g = grid_from([
        "#######",
        ".......",
        ".......",
        ".......",
        "#######",
    ])
    plain = astar(g, (1, 0), (1, 6), connectivity=8)          # start on top free row
    clear = astar(g, (1, 0), (1, 6), connectivity=8,
                  clearance_weight=5.0, inflation_radius=2)
    assert plain is not None and clear is not None
    # clearance field: middle row (2) is furthest from the # borders.
    cf = clearance_field(g)
    assert cf[2][3] > cf[1][3], "middle row should have more clearance"
    # the clearance-aware path should spend time on the center row (2); the plain
    # one has no reason to leave the straight top row.
    clear_rows = {r for (r, _) in clear}
    assert 2 in clear_rows, "clearance path should use the corridor center"
    print(f"PASS test_clearance_centers_path (plain rows={sorted({r for r,_ in plain})}, "
          f"clear rows={sorted(clear_rows)})")


def test_prune_collinear():
    straight = [(0, 0), (0, 1), (0, 2), (0, 3)]
    assert prune_collinear(straight) == [(0, 0), (0, 3)]
    print("PASS test_prune_collinear")


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for t in tests:
        t()
    print(f"\nAll {len(tests)} tests passed.")
