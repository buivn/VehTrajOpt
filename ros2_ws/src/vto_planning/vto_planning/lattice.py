"""State-lattice path search for a diff-drive robot — NO ROS dependencies.

See docs/22 for the full design. Same shape as astar.py (docs/07): framework-
free core, unit-testable on the host; a ROS node (lattice_planner_node.py, a
later step) is a thin adapter around this.

Unlike astar.py, the search state also carries a discretized HEADING, and
edges are a small library of precomputed MOTION PRIMITIVES (one straight run,
two curvature levels of left/right arc, two in-place pivots — 7 total)
instead of 8 fixed 45-degree grid moves, so an edge reflects how a diff-drive
robot actually moves rather than an arbitrary grid hop.

Units: like astar.py, this module is entirely CELL-based (radius/length in
cells, not meters) — a ROS node converts real robot parameters into cell
units using the map resolution, the same way `inflation_radius` already does
for A*.

Heading-index convention: N_HEADINGS bins over [0, 2*pi); heading index 0
means "pointing in the +row direction" — an arbitrary but fixed convention
(same idea as astar.py never knowing which way is "north"; a ROS node's
row/col <-> x/y mapping is what ties this to a real yaw).

Goal test is a TOLERANCE RADIUS around a (row, col) position (any final
heading accepted) rather than an exact lattice-state match: an arbitrary
requested goal cell is not generally reachable by an exact combination of
primitives (their endpoints land on specific offsets), so — like most
practical lattice planners — this searches for the cheapest state that gets
within `goal_tolerance` cells of the goal, not an exact hit. Tolerance 0.0
degenerates to an exact-position match.
"""
import heapq
import itertools
import math

from vto_planning.astar import clearance_field, is_free, octile, _reconstruct

N_HEADINGS = 16
HEADING_STEP = 2.0 * math.pi / N_HEADINGS


def heading_to_index(theta):
    """Nearest heading-bin index for a continuous heading (radians, any range)."""
    return int(round((theta % (2.0 * math.pi)) / HEADING_STEP)) % N_HEADINGS


def index_to_heading(h):
    """Continuous heading (radians, in [0, 2*pi)) for a heading-bin index."""
    return h * HEADING_STEP


def heading_index_from_yaw(yaw):
    """Heading-bin index for a REAL-WORLD yaw (TF/odometry convention: 0 along
    +x, counterclockwise toward +y). NOT the same as heading_to_index(yaw)
    directly -- this module's index convention is 0 along +row, rotating
    toward +col (primitives_at_heading), and a ROS node's row/col <-> x/y
    mapping is row<-y, col<-x (`_cell_from_world` in both planner nodes). So
    index 0 physically means +y (north), and the index angle is measured
    from the row/y-axis toward the col/x-axis -- the opposite rotation sense
    from standard math yaw. The correct conversion is a fixed pi/2 twist:
    lattice_angle = pi/2 - yaw (docs/39, found while debugging the Ackermann
    planner: a forward-only vehicle has no cheap way to correct a mislabeled
    start heading, unlike diff-drive's free in-place pivots, which is why
    this was invisible until now)."""
    return heading_to_index(math.pi / 2.0 - yaw)


def _sample_arc(kappa, length, n_samples=6):
    """Local-frame (start at origin, heading 0) samples of a constant-
    curvature arc: signed curvature `kappa` (1/radius; positive turns toward
    +col), arc length `length`. kappa=0 is a straight line. Returns a list of
    (drow, dcol, dtheta) offsets from the start — NOT yet rotated to a
    particular heading index or translated to a cell (see
    `primitives_at_heading`)."""
    pts = []
    for i in range(n_samples):
        s = length * i / (n_samples - 1)
        theta = kappa * s
        if abs(kappa) < 1e-9:
            dr, dc = s, 0.0
        else:
            dr = math.sin(theta) / kappa
            dc = (1.0 - math.cos(theta)) / kappa
        pts.append((dr, dc, theta))
    return pts


def make_primitive_library(step_cells=3.0, wide_radius_cells=6.0,
                            tight_radius_cells=3.0, pivot_cost_mult=2.5,
                            n_samples=6):
    """Build the canonical primitive set at heading index 0: 1 straight, 2
    curvature levels x 2 turn directions (4 arcs), 2 in-place pivots.

    Each primitive is a dict: `dh` (heading-index delta, int), `cost` (float,
    an execution-time proxy — distance for a moving primitive, a time
    penalty for a pivot), `samples` (list of (drow, dcol, dtheta) local
    offsets, not yet rotated/translated).

    Radii are chosen, not solved for: an arc's endpoint heading is whatever
    falls out of (radius, step_cells) and gets SNAPPED to the nearest heading
    bin — a known simplification versus solving an exact boundary-value
    problem per lattice edge (what a full state-lattice implementation, e.g.
    Pivtoraiko & Kelly, does); adequate for this project.
    """
    prims = [dict(dh=0, cost=step_cells,
                  samples=_sample_arc(0.0, step_cells, n_samples))]

    for radius in (wide_radius_cells, tight_radius_cells):
        for turn_sign in (+1.0, -1.0):
            pts = _sample_arc(turn_sign / radius, step_cells, n_samples)
            dh = int(round(pts[-1][2] / HEADING_STEP))
            if dh == 0:
                continue          # too gentle to change bin; redundant with straight
            prims.append(dict(dh=dh, cost=step_cells, samples=pts))

    for turn_sign in (+1, -1):
        theta_end = turn_sign * HEADING_STEP
        pts = [(0.0, 0.0, theta_end * i / (n_samples - 1)) for i in range(n_samples)]
        prims.append(dict(dh=turn_sign, cost=step_cells * pivot_cost_mult, samples=pts))

    return prims


def primitives_at_heading(canonical, heading_index):
    """Rotate the canonical (heading-0) primitive set to `heading_index`.
    Only (drow, dcol) rotates; `dh` (a heading CHANGE) and `cost` are
    rotation-invariant. Adds the integer (dr, dc) grid offset used for graph
    expansion, alongside the rotated continuous `samples` used for
    collision-checking / path densification."""
    angle = heading_index * HEADING_STEP
    ca, sa = math.cos(angle), math.sin(angle)
    out = []
    for p in canonical:
        rotated = [(dr * ca - dc * sa, dr * sa + dc * ca, dtheta)
                   for (dr, dc, dtheta) in p["samples"]]
        end_dr, end_dc, _ = rotated[-1]
        out.append(dict(dh=p["dh"], cost=p["cost"], samples=rotated,
                         dr=int(round(end_dr)), dc=int(round(end_dc))))
    return out


def build_primitive_table(canonical=None, **kwargs):
    """Precompute `primitives_at_heading` for every heading index once — the
    'generated once at startup, looked up during search' table from docs/22.
    Build this once outside the search (and outside any per-tick loop in the
    eventual ROS node), not per `lattice()` call."""
    if canonical is None:
        canonical = make_primitive_library(**kwargs)
    return [primitives_at_heading(canonical, h) for h in range(N_HEADINGS)]


def _primitive_is_clear(grid, row, col, primitive, occ_thresh):
    """A curved primitive can clip a wall mid-arc without either endpoint
    being occupied — check every sampled pose along it, not just the
    destination cell (unlike astar.py's single-cell move check)."""
    for (dr, dc, _theta) in primitive["samples"][1:]:   # [0] is (0,0,*), already free
        if not is_free(grid, row + int(round(dr)), col + int(round(dc)), occ_thresh):
            return False
    return True


def lattice(grid, start, goal, primitive_table=None, goal_tolerance=1.5,
            occ_thresh=50, clearance_weight=0.0, inflation_radius=0, clearance=None,
            max_expansions=None):
    """Find a least-cost lattice path from `start` to `goal`.

    Args:
        grid: 2D occupancy grid, same convention as astar.py.
        start: (row, col, heading_index) — full lattice state.
        goal: (row, col) — position only; any final heading is accepted.
        primitive_table: from `build_primitive_table()`; built with defaults
            if omitted (do this once outside a hot loop in real use).
        goal_tolerance: accept any state within this many cells of `goal`
            (Euclidean) — see module docstring for why this isn't exact-match.
        clearance_weight, inflation_radius, clearance: same meaning as astar().
        max_expansions: cap on states popped off the open set before giving
            up early (same return as genuine infeasibility: None). Default
            None = unlimited, the original behavior — a REAL infeasible
            query (e.g. a goal only reachable by reversing out of a
            forward-only dead-end, docs/40) has no path to find no matter
            how much of the graph gets expanded, so it exhausts the ENTIRE
            reachable state space (rows*cols*N_HEADINGS in the worst case)
            before returning None — measured multi-minute-plus on the maze
            map. A caller doing many such queries (e.g. a benchmark's tour
            builder, sampling and rejecting candidates) should pass a finite
            budget; the live planner node currently doesn't, since a bad
            interactive goal is rare and one slow reject beats a wrong path.
        """
    if not is_free(grid, start[0], start[1], occ_thresh) or not is_free(grid, *goal, occ_thresh):
        return None
    if primitive_table is None:
        primitive_table = build_primitive_table()

    if clearance_weight > 0.0 and inflation_radius > 0:
        if clearance is None:
            clearance = clearance_field(grid, occ_thresh)

        def cell_penalty(r, c):
            return clearance_weight * max(0, inflation_radius - clearance[r][c])

        def penalty(row, col, prim):
            # Integrated over the primitive's whole sampled path (same points
            # _primitive_is_clear already walks), not just its endpoint --
            # and scaled by the primitive's own length (prim["cost"], cell
            # units), so total penalty is proportional to distance actually
            # travelled near a wall. The endpoint-only version charged the
            # SAME one-time penalty regardless of how far a primitive
            # reached, which made the search prefer fewer, longer primitives
            # through any corridor narrower than inflation_radius -- even
            # diagonal zigzag ones -- over many short straight ones covering
            # the identical distance. Confirmed both by tracing a real
            # planned path (an exact sawtooth of +-45 deg primitives holding
            # one column fixed through an ordinary corridor) and by it
            # showing up under two independent controllers (Pure Pursuit and
            # MPC) tracking the same /plan -- a planning defect, not either
            # controller's.
            total, n = 0.0, 0
            for (dr, dc, _theta) in prim["samples"][1:]:
                total += cell_penalty(row + int(round(dr)), col + int(round(dc)))
                n += 1
            avg = (total / n) if n else 0.0
            return avg * prim["cost"]
    else:
        def penalty(row, col, prim):
            return 0.0

    def heuristic(state):
        # minus goal_tolerance keeps this admissible: the true remaining cost
        # can be less than the raw distance once a state within tolerance
        # counts as reaching the goal.
        return max(0.0, octile((state[0], state[1]), goal) - goal_tolerance)

    def reached(state):
        return math.hypot(state[0] - goal[0], state[1] - goal[1]) <= goal_tolerance

    counter = itertools.count()
    open_heap = [(heuristic(start), next(counter), start)]
    g_score = {start: 0.0}
    came_from = {}
    closed = set()
    expansions = 0

    while open_heap:
        _, _, current = heapq.heappop(open_heap)
        if reached(current):
            return _reconstruct(came_from, current)
        if current in closed:
            continue
        closed.add(current)
        expansions += 1
        if max_expansions is not None and expansions > max_expansions:
            return None

        row, col, heading = current
        for prim in primitive_table[heading]:
            nr, nc = row + prim["dr"], col + prim["dc"]
            if not is_free(grid, nr, nc, occ_thresh):
                continue
            if not _primitive_is_clear(grid, row, col, prim, occ_thresh):
                continue
            neighbor = (nr, nc, (heading + prim["dh"]) % N_HEADINGS)
            if neighbor in closed:
                continue
            tentative = g_score[current] + prim["cost"] + penalty(row, col, prim)
            if tentative < g_score.get(neighbor, math.inf):
                g_score[neighbor] = tentative
                came_from[neighbor] = current
                heapq.heappush(open_heap, (tentative + heuristic(neighbor), next(counter), neighbor))

    return None


def _find_primitive(prims, row, col, heading, next_state):
    nr, nc, nh = next_state
    for p in prims:
        if (row + p["dr"], col + p["dc"], (heading + p["dh"]) % N_HEADINGS) == (nr, nc, nh):
            return p
    raise ValueError("no primitive connects the given consecutive lattice states")


def densify_path(states, primitive_table):
    """Expand a lattice state path into the dense (row, col) trace each
    primitive actually swept — a curved segment handed to the tracker as a
    single chord would repeat docs/10's corner-cutting problem, so this keeps
    every sampled point, not just primitive endpoints.

    Returns list[(row, col)] (floats — deliberately not rounded to integer
    cells, which would flatten a smooth arc back into a staircase), start to
    goal inclusive.
    """
    if not states:
        return []
    if len(states) == 1:
        return [(float(states[0][0]), float(states[0][1]))]

    dense = [(float(states[0][0]), float(states[0][1]))]
    for i in range(len(states) - 1):
        row, col, heading = states[i]
        prim = _find_primitive(primitive_table[heading], row, col, heading, states[i + 1])
        for (dr, dc, _theta) in prim["samples"][1:]:
            dense.append((row + dr, col + dc))
    return dense
