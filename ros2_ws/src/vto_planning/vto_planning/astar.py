"""Pure A* grid search — NO ROS dependencies.

Kept framework-free on purpose (see docs/07): this module takes an occupancy
grid + start/goal cells and returns a list of cells. That means it is unit-
testable on the host (no colcon, no container, no Gazebo) and reusable anywhere.
The ROS node (astar_planner_node.py) is a thin adapter around this.

Grid convention (matches nav_msgs/OccupancyGrid values):
    value <  0            -> unknown  -> treated as OCCUPIED (safe default)
    0  <= value < thresh  -> free
    value >= thresh       -> occupied
A "cell" is a (row, col) tuple: hashable, so usable as dict/set keys.
"""
import heapq
import itertools
import math
from collections import deque

SQRT2 = math.sqrt(2.0)

# 4-connected moves (row, col, step_cost) and the diagonal add-ons for 8-conn.
_MOVES4 = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0)]
_MOVES8 = _MOVES4 + [(-1, -1, SQRT2), (-1, 1, SQRT2), (1, -1, SQRT2), (1, 1, SQRT2)]


def is_free(grid, r, c, occ_thresh=50):
    """True if (r, c) is in-bounds and free (not occupied, not unknown)."""
    if r < 0 or c < 0 or r >= len(grid) or c >= len(grid[0]):
        return False
    v = grid[r][c]
    return 0 <= v < occ_thresh


def octile(a, b):
    """Admissible 8-connected heuristic: cost of the cheapest obstacle-free path
    on an empty grid. dmax straight + (sqrt2-1)*dmin diagonal steps."""
    dr = abs(a[0] - b[0])
    dc = abs(a[1] - b[1])
    return (dr + dc) + (SQRT2 - 2.0) * min(dr, dc)


def manhattan(a, b):
    """Admissible 4-connected heuristic."""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def clearance_field(grid, occ_thresh=50):
    """Multi-source BFS distance transform: for every free cell, the number of
    (4-connected) steps to the nearest obstacle. Occupied/unknown cells = 0.

    This is the 'distance to nearest wall' used by the clearance penalty. A pure-
    BFS (Manhattan) transform keeps this module numpy-free; it is an approximation
    of the Euclidean distance transform but is monotone and plenty for a penalty.
    """
    rows, cols = len(grid), len(grid[0])
    INF = rows + cols + 1
    dist = [[INF] * cols for _ in range(rows)]
    q = deque()
    for r in range(rows):
        for c in range(cols):
            if not (0 <= grid[r][c] < occ_thresh):  # obstacle or unknown = source
                dist[r][c] = 0
                q.append((r, c))
    while q:
        r, c = q.popleft()
        for dr, dc, _ in _MOVES4:
            nr, nc = r + dr, c + dc
            if 0 <= nr < rows and 0 <= nc < cols and dist[nr][nc] > dist[r][c] + 1:
                dist[nr][nc] = dist[r][c] + 1
                q.append((nr, nc))
    return dist


def inflate_obstacles(grid, radius_cells, occ_thresh=50):
    """Configuration-space inflation: grow every obstacle by `radius_cells` so a
    POINT robot on the returned grid is equivalent to the real robot (radius) on
    the original. Any free cell within `radius_cells` steps of a wall becomes
    occupied. This is a HARD guarantee (unlike the soft clearance cost): the
    planner physically cannot route the body through a gap it wouldn't fit, and
    keeps `radius_cells` clearance from every wall including corners.

    Returns a NEW grid (0 free / 100 occupied); the input is not mutated.
    """
    if radius_cells <= 0:
        return [list(row) for row in grid]
    dist = clearance_field(grid, occ_thresh)   # steps to nearest wall (0 on walls)
    out = []
    for r, row in enumerate(grid):
        new_row = []
        for c, v in enumerate(row):
            occupied = not (0 <= v < occ_thresh) or dist[r][c] <= radius_cells
            new_row.append(100 if occupied else 0)
        out.append(new_row)
    return out


def _reconstruct(came_from, current):
    """Walk parent links from goal back to start, then reverse."""
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path


def astar(grid, start, goal, connectivity=8, clearance_weight=0.0,
          inflation_radius=0, occ_thresh=50, clearance=None):
    """Find a least-cost path from start to goal on an occupancy grid.

    Args:
        grid: 2D sequence of ints (OccupancyGrid convention above).
        start, goal: (row, col) tuples.
        connectivity: 4 or 8.
        clearance_weight (w): >0 adds a wall-proximity penalty to each step so
            paths ride corridor centers. penalty(cell) = w * max(0, R - dist).
        inflation_radius (R): cells within R of a wall are penalized (in cells).
        occ_thresh: value at/above which a cell counts as occupied.
        clearance: optional precomputed clearance_field(grid); pass it to avoid
            recomputing the BFS distance transform on every plan (the ROS node
            computes it once when the map arrives).

    Returns:
        list[(row, col)] from start to goal inclusive, or None if unreachable.
    """
    if not is_free(grid, *start) or not is_free(grid, *goal):
        return None

    moves = _MOVES8 if connectivity == 8 else _MOVES4
    h = octile if connectivity == 8 else manhattan

    if clearance_weight > 0.0 and inflation_radius > 0:
        if clearance is None:
            clearance = clearance_field(grid, occ_thresh)

        def penalty(r, c):
            return clearance_weight * max(0, inflation_radius - clearance[r][c])
    else:
        def penalty(r, c):
            return 0.0

    counter = itertools.count()          # tie-break: keeps heap from comparing cells
    open_heap = [(h(start, goal), next(counter), start)]
    g_score = {start: 0.0}
    came_from = {}
    closed = set()

    while open_heap:
        _, _, current = heapq.heappop(open_heap)
        if current == goal:
            return _reconstruct(came_from, current)
        if current in closed:            # stale duplicate (no decrease-key in heapq)
            continue
        closed.add(current)

        r, c = current
        for dr, dc, step in moves:
            nr, nc = r + dr, c + dc
            if not is_free(grid, nr, nc, occ_thresh):
                continue
            # no corner-cutting: a diagonal move needs both shared orthogonals free
            if dr != 0 and dc != 0:
                if not is_free(grid, r, nc, occ_thresh) or not is_free(grid, nr, c, occ_thresh):
                    continue
            neighbor = (nr, nc)
            if neighbor in closed:
                continue
            tentative = g_score[current] + step + penalty(nr, nc)
            if tentative < g_score.get(neighbor, math.inf):   # relax
                g_score[neighbor] = tentative
                came_from[neighbor] = current
                f = tentative + h(neighbor, goal)
                heapq.heappush(open_heap, (f, next(counter), neighbor))

    return None                          # open set exhausted -> goal unreachable


def prune_collinear(path):
    """Drop interior points that lie on a straight run, so the follower gets a
    compact path (fewer, meaningful waypoints). Purely geometric, keeps endpoints."""
    if path is None or len(path) < 3:
        return path
    out = [path[0]]
    for i in range(1, len(path) - 1):
        (r0, c0), (r1, c1), (r2, c2) = out[-1], path[i], path[i + 1]
        # cross product of (mid-prev) x (next-mid); 0 => collinear => drop mid
        if (r1 - r0) * (c2 - c1) - (c1 - c0) * (r2 - r1) != 0:
            out.append(path[i])
    out.append(path[-1])
    return out
