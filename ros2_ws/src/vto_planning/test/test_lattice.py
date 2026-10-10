"""Host-runnable tests for the pure lattice module (no ROS needed).

Run directly:   python3 test/test_lattice.py
Or with pytest: pytest test/test_lattice.py
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from vto_planning.lattice import (  # noqa: E402
    N_HEADINGS, HEADING_STEP, build_primitive_table, densify_path,
    heading_to_index, index_to_heading, lattice, make_primitive_library,
    _primitive_is_clear,
)

FREE, OCC = 0, 100


def grid_from(rows):
    """Build a grid from an ASCII map: '#' occupied, '.' free."""
    return [[OCC if ch == "#" else FREE for ch in row] for row in rows]


def test_straight_line():
    # heading index 4 (90 deg) points along +col in this module's convention.
    g = grid_from(["." * 22])
    table = build_primitive_table()
    path = lattice(g, (0, 0, 4), (0, 21), primitive_table=table, goal_tolerance=0.6)
    assert path is not None
    assert (path[0][0], path[0][1]) == (0, 0)
    assert (path[-1][0], path[-1][1]) == (0, 21)
    print(f"PASS test_straight_line (states={len(path)})")


def test_goes_around_wall():
    #  a wall splits the room; must go around via the open column on the right
    g = grid_from([
        "..........",
        "..........",
        "..........",
        "########..",
        "..........",
        "..........",
        "..........",
    ])
    table = build_primitive_table()
    path = lattice(g, (0, 0, 4), (6, 0), primitive_table=table, goal_tolerance=1.5)
    assert path is not None, "should find a way around the wall"
    dense = densify_path(path, table)
    for (r, c) in dense:
        rr, cc = int(round(r)), int(round(c))
        assert g[rr][cc] == FREE, f"dense path point ({rr},{cc}) enters an occupied cell"
    print(f"PASS test_goes_around_wall (states={len(path)}, dense_pts={len(dense)})")


def test_unreachable_returns_none():
    g = grid_from([
        "....#.....",
        "....#.....",
        "....#.....",
        "....#.....",
        "....#.....",
    ])
    table = build_primitive_table()
    path = lattice(g, (2, 0, 4), (2, 9), primitive_table=table, goal_tolerance=0.6)
    assert path is None, "a floor-to-ceiling wall must block the goal"
    print("PASS test_unreachable_returns_none")


def test_primitive_is_clear_catches_midarc_obstacle():
    canonical = make_primitive_library()
    table = build_primitive_table(canonical)
    # find a curved (non-straight, non-pivot) primitive at heading 0
    arc = next(p for p in table[0] if p["dh"] != 0 and (p["dr"] != 0 or p["dc"] != 0))
    mid_dr, mid_dc, _ = arc["samples"][len(arc["samples"]) // 2]
    mid_r, mid_c = int(round(mid_dr)), int(round(mid_dc))

    clear_grid = grid_from(["." * 10 for _ in range(10)])
    assert _primitive_is_clear(clear_grid, 3, 3, arc, occ_thresh=50)

    blocked_grid = [row[:] for row in clear_grid]
    blocked_grid[3 + mid_r][3 + mid_c] = OCC
    assert not _primitive_is_clear(blocked_grid, 3, 3, arc, occ_thresh=50), \
        "an obstacle at a mid-arc sample must invalidate the primitive"
    print("PASS test_primitive_is_clear_catches_midarc_obstacle")


def test_pivot_primitives_have_zero_displacement():
    canonical = make_primitive_library()
    pivots = [p for p in canonical if p["dh"] != 0 and p["samples"][-1][0] == 0.0
              and p["samples"][-1][1] == 0.0]
    assert len(pivots) == 2, "expected one left and one right in-place pivot"
    for p in pivots:
        assert abs(p["dh"]) == 1
    print("PASS test_pivot_primitives_have_zero_displacement")


def test_heading_index_roundtrip():
    for h in range(N_HEADINGS):
        theta = index_to_heading(h)
        assert heading_to_index(theta) == h
    # a small perturbation should still snap back to the nearest bin
    theta = index_to_heading(3) + 0.01
    assert heading_to_index(theta) == 3
    print("PASS test_heading_index_roundtrip")


def test_densify_path_adds_points_for_turn():
    table = build_primitive_table()
    # heading 0 -> pick a turning primitive's resulting state as a 2-state path
    turn = next(p for p in table[0] if p["dh"] != 0)
    start = (5, 5, 0)
    end = (5 + turn["dr"], 5 + turn["dc"], (0 + turn["dh"]) % N_HEADINGS)
    dense = densify_path([start, end], table)
    assert dense[0] == (5.0, 5.0)
    assert (round(dense[-1][0]), round(dense[-1][1])) == (end[0], end[1])
    assert len(dense) > 2, "a turning primitive should contribute intermediate points"
    print(f"PASS test_densify_path_adds_points_for_turn (pts={len(dense)})")


def test_make_primitive_library_has_seven():
    prims = make_primitive_library()
    assert len(prims) == 7, f"expected 1 straight + 4 arcs + 2 pivots, got {len(prims)}"
    print("PASS test_make_primitive_library_has_seven")


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for t in tests:
        t()
    print(f"\nAll {len(tests)} tests passed.")
