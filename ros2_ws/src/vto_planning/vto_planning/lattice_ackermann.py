"""Ackermann (car-like) state-lattice control set + search — step 2 (docs/25),
reverse motion added step 2 of docs/40-41's follow-on. NO ROS dependencies.

Builds on dubins.py (docs/24) / reeds_shepp.py (docs/41, the edge primitive)
and reuses lattice.py (docs/22, the search) unchanged. Pipeline:

  1. enumerate candidate lattice endpoints (drow, dcol, dheading_bin) in a
     window around the origin;
  2. connect each to pose (0,0,0) with a Dubins (forward-only) or
     Reeds-Shepp (reverse-capable) shortest path (cell units, radius =
     turning_radius_cells);
  3. drop the over-long ones (a primitive that loops halfway round the
     turning circle is not a unit move);
  4. greedily prune the redundant ones — a candidate whose endpoint node is
     already reachable by cheaper accepted primitives adds nothing;

what survives is emitted in the EXACT dict shape make_primitive_library()
produces, so build_primitive_table() + lattice() run byte-for-byte the same
as for the diff-drive lattice.

reverse=False (default) is the ORIGINAL, still-live behavior: Dubins-only,
heading swing capped at max_heading_change_bins (default 4 = 90 deg) per
primitive. reverse=True switches to Reeds-Shepp and widens that cap to
N_HEADINGS//2 (180 deg) by default -- deliberately, not an oversight: docs/40
proved that capping heading swing at 90 deg is exactly what leaves a
forward-only car with no way to escape certain headings (a K-turn IS a
~180 deg reorientation), so a reverse-enabled library that kept the old cap
would still miss the primitives that actually fix the trap. reverse=True is
NOT wired into lattice_ackermann_planner_node.py's default yet -- see docs/42
(step 2 write-up) for why that's a deliberate boundary, not a TODO: the
controller (pure_pursuit_ackermann_node.py) has no way to execute a cusp yet
(docs/41 step 3), and this project has already been burned once by enabling
a new degree of freedom before the consuming system was ready for it
(docs/14 Sec 6's MPC reverse bug -- "nothing penalized localization-unfriendly
motion" once v_min<0 was allowed). Building this behind an opt-in default
avoids repeating that.

Heading convention: identical to lattice.py (N_HEADINGS bins over [0, 2*pi),
index 0 along +row, +heading rotates toward +col). Dubins/Reeds-Shepp run in
(x = row, y = col) with yaw measured the same way, and every primitive is
connected FROM (0,0,0) so a sample's absolute yaw equals its heading change
from the primitive start — matching lattice.py's samples[i][2] convention.
"""
import heapq
import math

from vto_planning.dubins import dubins_shortest_path, mod2pi, sample_dubins
from vto_planning.reeds_shepp import reeds_shepp_shortest_path, sample_reeds_shepp
from vto_planning.lattice import (
    HEADING_STEP, N_HEADINGS, build_primitive_table, index_to_heading, lattice,
)


def _rotate_int(dr, dc, heading_index):
    """(dr, dc) rotated to `heading_index` and rounded to integer cells — the
    same rotation lattice.py's primitives_at_heading uses for composition."""
    ang = heading_index * HEADING_STEP
    ca, sa = math.cos(ang), math.sin(ang)
    return int(round(dr * ca - dc * sa)), int(round(dr * sa + dc * ca))


def _reachable_by_combo(accepted, target, budget, max_depth=4, pos_tol=1,
                        max_expansions=6000):
    """True iff `target` = (dr, dc, dh) is (near-)reachable from (0,0,0) by
    composing at most `max_depth` already-accepted primitives for total cost
    <= `budget` and ending within `pos_tol` cells of the target position at
    the exact target heading.

    A small Dijkstra over primitive compositions. Two deliberate slacknesses,
    without which almost nothing prunes for a Dubins lattice:
      - `budget` carries a cost slack (redundancy_tol) — a direct Dubins curve
        is ALWAYS <= any composition (triangle inequality), so exact-cost
        redundancy essentially never occurs; we accept a slightly suboptimal
        composition to kill a primitive and shrink the branching factor;
      - `pos_tol` absorbs the integer rounding at each composition join.
    On the expansion cap it returns False (keep the primitive) — the safe side.
    """
    tr, tc, th = target
    start = (0, 0, 0)
    best = {start: 0.0}
    frontier = [(0.0, 0, start)]
    expansions = 0
    while frontier:
        cost, depth, (r, c, h) = heapq.heappop(frontier)
        if (depth >= 1 and h == th and cost <= budget
                and max(abs(r - tr), abs(c - tc)) <= pos_tol):
            return True         # depth >= 1: the empty composition doesn't count
        if depth >= max_depth or cost > budget:
            continue
        expansions += 1
        if expansions > max_expansions:
            return False
        for (plen, pdr, pdc, pdh, _path) in accepted:
            new_cost = cost + plen
            if new_cost > budget:
                continue
            rr, cc = _rotate_int(pdr, pdc, h)
            nxt = (r + rr, c + cc, (h + pdh) % N_HEADINGS)
            if new_cost < best.get(nxt, math.inf):
                best[nxt] = new_cost
                heapq.heappush(frontier, (new_cost, depth + 1, nxt))
    return False


def make_ackermann_primitive_library(turning_radius_cells=4.0,
                                     max_reach_cells=7,
                                     max_heading_change_bins=4,
                                     max_primitive_len_cells=None,
                                     samples_per_cell=2.5,
                                     redundancy_tol=0.15,
                                     combo_pos_tol=1,
                                     max_combo_depth=4,
                                     reverse=False,
                                     reverse_max_heading_change_bins=None,
                                     reverse_reach_cells=3):
    """Enumerate -> connect (Dubins, or Dubins+Reeds-Shepp) -> length-filter
    -> greedily prune.

    Returns list[dict(dh, cost, samples)] — the same shape lattice.py's
    make_primitive_library() returns, ready for build_primitive_table().
    `samples` are (drow, dcol, dtheta) offsets from the origin at heading 0
    (direction of travel through a sample -- forward or backward -- is NOT
    encoded here; a primitive is opaque cost+geometry to lattice.py either
    way. See this module's docstring for why that's fine for search/
    collision-checking but not yet enough to hand a controller a path).

    reverse=False (default): behavior is BYTE-FOR-BYTE unchanged from
    before reverse existed -- max_heading_change_bins=4 (90 deg), Dubins
    only, `reverse_*` params unused.

    reverse=True adds a SECOND, SEPARATE enumeration pass over WIDE heading
    changes (> max_heading_change_bins, up to reverse_max_heading_change_bins
    -- default N_HEADINGS//2 = 180 deg), connected with Reeds-Shepp instead
    of Dubins. Deliberately NOT just "run the whole enumeration through
    Reeds-Shepp with a wider cap": measured cost of that first attempt was
    45s to build + 7-12s per search query (docs/42) -- prohibitive for
    anything that runs this check repeatedly (e.g. a benchmark tour
    builder). The fix follows directly from the physics: a big
    reorientation (what wide-heading candidates are for) doesn't need to
    also cover distance -- that's what the narrow, cheap Dubins primitives
    already do. So the wide/reverse pass uses its OWN, much smaller reach
    (reverse_reach_cells, default 3 vs the narrow pass's default 7) --
    quadratically fewer candidates hit the expensive Newton solve, and the
    resulting primitives are shorter (fewer collision-check samples),
    which is also most of why per-query search got faster, not just table
    build (docs/42's measurements).
    """  # noqa: E501 (Newton, not "Newal" -- typo fixed in the paragraph above)
    R = float(turning_radius_cells)
    if max_primitive_len_cells is None:
        max_primitive_len_cells = 1.1 * math.pi * R      # allows a U-turn, not a full loop
    if reverse_max_heading_change_bins is None:
        reverse_max_heading_change_bins = N_HEADINGS // 2

    def _enumerate(reach, dh_min, dh_max, use_reverse):
        out = []
        for dr in range(-reach, reach + 1):
            for dc in range(-reach, reach + 1):
                if dr == 0 and dc == 0:
                    continue
                for dh in range(N_HEADINGS):
                    wrapped = dh if dh <= N_HEADINGS // 2 else dh - N_HEADINGS
                    if not (dh_min <= abs(wrapped) <= dh_max):
                        continue
                    goal = (float(dr), float(dc), index_to_heading(dh))
                    if use_reverse:
                        path = reeds_shepp_shortest_path((0.0, 0.0, 0.0), goal, R)
                    else:
                        path = dubins_shortest_path((0.0, 0.0, 0.0), goal, R)
                    if path.length > max_primitive_len_cells:
                        continue
                    out.append((path.length, dr, dc, dh, path))
        return out

    # 1 + 2: narrow heading changes -> Dubins, full reach (unchanged).
    candidates = _enumerate(int(max_reach_cells), 0, max_heading_change_bins,
                            use_reverse=False)
    # wide heading changes (reverse only) -> Reeds-Shepp, LOCAL reach.
    if reverse:
        candidates += _enumerate(int(reverse_reach_cells), max_heading_change_bins + 1,
                                 reverse_max_heading_change_bins, use_reverse=True)

    # 4: shortest-first greedy prune
    candidates.sort(key=lambda t: t[0])
    accepted = []                       # (length, dr, dc, dh, path)
    for length, dr, dc, dh, path in candidates:
        budget = length * (1.0 + redundancy_tol)
        if _reachable_by_combo(accepted, (dr, dc, dh), budget,
                               max_depth=max_combo_depth, pos_tol=combo_pos_tol):
            continue
        accepted.append((length, dr, dc, dh, path))

    # sample the survivors into lattice.py's primitive dict shape. `path` is
    # a DubinsPath (narrow pass) or ReedsSheppPath (wide/reverse pass) --
    # dispatch per-candidate, not by one global flag, since reverse=True
    # produces a MIX of both from the two enumeration passes above.
    step = 1.0 / float(samples_per_cell)
    library = []
    for length, dr, dc, dh, path in accepted:
        sampler = sample_dubins if hasattr(path, "word") else sample_reeds_shepp
        samples = [(x, y, mod2pi(yaw)) for (x, y, yaw) in sampler(path, step)]
        library.append(dict(dh=dh, cost=length, samples=samples))
    return library


def ackermann_primitive_table(**kwargs):
    """build_primitive_table() applied to the Ackermann control set — the
    'generated once at startup, looked up during search' table. Build once,
    outside any per-tick loop."""
    return build_primitive_table(make_ackermann_primitive_library(**kwargs))


def plan_ackermann(grid, start, goal, primitive_table=None, turning_radius_cells=4.0,
                   goal_tolerance=1.5, occ_thresh=50, clearance_weight=0.0,
                   inflation_radius=0, clearance=None, reverse=False):
    """Convenience wrapper: build the control-set table if not supplied, then
    run lattice.py's search. `start` is (row, col, heading_index); `goal` is
    (row, col). Returns list[(row, col, heading_index)] or None."""
    if primitive_table is None:
        primitive_table = ackermann_primitive_table(
            turning_radius_cells=turning_radius_cells, reverse=reverse)
    return lattice(grid, start, goal, primitive_table=primitive_table,
                   goal_tolerance=goal_tolerance, occ_thresh=occ_thresh,
                   clearance_weight=clearance_weight, inflation_radius=inflation_radius,
                   clearance=clearance)
