#!/usr/bin/env python3
"""Generate a Gazebo (SDF) maze world with uniform-width corridors.

A perfect maze (recursive-backtracker / DFS) on a grid, emitted as static wall
boxes inside a bounded rectangle. Corridors are a fixed width; wall thickness is
solved so the grid fills the requested outer dimensions *exactly*.

Defaults match VehTrajOpt's request: 20 m x 30 m, origin at the bottom-left,
corridors = 5 x robot width (0.30 m) = 1.50 m.

Usage:
  python3 generate_maze.py --out ../worlds/maze.sdf
  python3 generate_maze.py --width 20 --height 30 --corridor 1.5 --seed 7
"""
import argparse
import math


def solve_grid(length, corridor, wall_nominal):
    """Pick an integer corridor count and the exact wall thickness that makes
    n*corridor + (n+1)*wall == length (uniform corridors, exact fit)."""
    n = max(1, round((length - wall_nominal) / (corridor + wall_nominal)))
    t = (length - n * corridor) / (n + 1)
    return n, t


def carve(nx, ny, seed):
    """Recursive-backtracker maze. Returns the sets of interior walls that REMAIN.

    vwalls: (k, j) = vertical wall on grid line k (1..nx-1), row j  -> between
            cells (k-1, j) and (k, j).
    hwalls: (i, k) = horizontal wall on grid line k (1..ny-1), column i -> between
            cells (i, k-1) and (i, k).
    """
    # deterministic PRNG (avoid importing random for reproducibility clarity)
    state = (seed * 2654435761 + 12345) & 0xFFFFFFFF

    def rnd(n):
        nonlocal state
        state = (1103515245 * state + 12345) & 0x7FFFFFFF
        return state % n

    vwalls = {(k, j) for k in range(1, nx) for j in range(ny)}
    hwalls = {(i, k) for i in range(nx) for k in range(1, ny)}

    visited = {(0, 0)}
    stack = [(0, 0)]
    while stack:
        i, j = stack[-1]
        nbrs = []
        if i > 0 and (i - 1, j) not in visited:
            nbrs.append(("W", i - 1, j))
        if i < nx - 1 and (i + 1, j) not in visited:
            nbrs.append(("E", i + 1, j))
        if j > 0 and (i, j - 1) not in visited:
            nbrs.append(("S", i, j - 1))
        if j < ny - 1 and (i, j + 1) not in visited:
            nbrs.append(("N", i, j + 1))
        if not nbrs:
            stack.pop()
            continue
        d, ni, nj = nbrs[rnd(len(nbrs))]
        if d == "E":
            vwalls.discard((i + 1, j))
        elif d == "W":
            vwalls.discard((i, j))
        elif d == "N":
            hwalls.discard((i, j + 1))
        elif d == "S":
            hwalls.discard((i, j))
        visited.add((ni, nj))
        stack.append((ni, nj))
    return vwalls, hwalls


def _cell_graph(nx, ny, vwalls, hwalls):
    """Adjacency list over the (already-carved) cell grid: cell (i,j) ->
    list of open neighbors, per the current vwalls/hwalls (walls that
    remain closed)."""
    adj = {(i, j): [] for i in range(nx) for j in range(ny)}
    for i in range(nx):
        for j in range(ny):
            if i + 1 < nx and (i + 1, j) not in vwalls:
                adj[(i, j)].append((i + 1, j))
                adj[(i + 1, j)].append((i, j))
            if j + 1 < ny and (i, j + 1) not in hwalls:
                adj[(i, j)].append((i, j + 1))
                adj[(i, j + 1)].append((i, j))
    return adj


def _bfs_dist(adj, source):
    """Hop-count distance from `source` to every reachable cell (a perfect
    maze is one connected component, so this covers the whole grid)."""
    dist = {source: 0}
    frontier = [source]
    while frontier:
        nxt = []
        for cell in frontier:
            d = dist[cell]
            for nb in adj[cell]:
                if nb not in dist:
                    dist[nb] = d + 1
                    nxt.append(nb)
        frontier = nxt
    return dist


def carve_rooms(vwalls, hwalls, nx, ny, num_rooms, room_cells, min_sep_cells, seed):
    """Open up `num_rooms` square blocks of `room_cells` x `room_cells` cells
    into fully wall-free rooms -- large enough for the robot to turn freely
    in any direction, so a forward-only planner is never trapped there
    (docs/40/43) -- spread out by requiring each newly-chosen room's grid
    (hop-count) distance to every already-chosen room exceed
    `min_sep_cells`. Graph distance, not straight-line: a straight-line-far
    pair can still be a short corridor hop apart if a wall between them was
    carved away, which wouldn't satisfy "travel distance longer than a
    threshold" the way it's actually meant.

    Removing walls only ever ADDS connectivity to an already-fully-connected
    spanning tree (docs/43), so this can't disconnect anything -- no need to
    re-validate reachability after.

    Returns (vwalls, hwalls, rooms) where rooms is a list of dicts
    (anchor_i, anchor_j, cells, center_i, center_j) in cell-index units
    (fractional cell coords for the center, since a block's center generally
    isn't itself a cell).
    """
    vwalls, hwalls = set(vwalls), set(hwalls)
    if num_rooms <= 0:
        return vwalls, hwalls, []

    adj = _cell_graph(nx, ny, vwalls, hwalls)
    candidates = [(i, j) for i in range(nx - room_cells + 1)
                  for j in range(ny - room_cells + 1)]

    state = (seed * 2246822519 + 3266489917) & 0xFFFFFFFF

    def rnd(n):
        nonlocal state
        state = (1103515245 * state + 12345) & 0x7FFFFFFF
        return state % n

    # shuffle candidates (Fisher-Yates with the same hand-rolled LCG carve()
    # uses, for the same reproducibility-without-`random` reason)
    for k in range(len(candidates) - 1, 0, -1):
        m = rnd(k + 1)
        candidates[k], candidates[m] = candidates[m], candidates[k]

    def _blocks_touch_or_overlap(a1, a2, margin=2):
        """True if the two room_cells-square blocks anchored at a1, a2 come
        within `margin` grid cells of each other in BOTH axes -- i.e. close
        enough that opening both would merge them into one blob (or leave
        no real corridor between them). Two axis-aligned squares are safely
        apart if they're separated by >= margin in AT LEAST ONE axis."""
        x_gap = max(a1[0], a2[0]) - min(a1[0] + room_cells, a2[0] + room_cells)
        y_gap = max(a1[1], a2[1]) - min(a1[1] + room_cells, a2[1] + room_cells)
        return not (x_gap >= margin or y_gap >= margin)

    chosen = []       # list of (anchor_i, anchor_j)
    chosen_centers = []
    for (ai, aj) in candidates:
        if len(chosen) >= num_rooms:
            break
        if any(_blocks_touch_or_overlap((ai, aj), a) for a in chosen):
            continue          # would merge with an already-chosen room -- skip
        center = (ai, aj)   # BFS distance from the anchor cell is a fine proxy
        dists = _bfs_dist(adj, center)
        if all(dists.get(c, math.inf) >= min_sep_cells for c in chosen_centers):
            chosen.append((ai, aj))
            chosen_centers.append(center)

    for (ai, aj) in chosen:
        for i in range(ai, ai + room_cells):
            for j in range(aj, aj + room_cells):
                if i + 1 < ai + room_cells:
                    vwalls.discard((i + 1, j))
                if j + 1 < aj + room_cells:
                    hwalls.discard((i, j + 1))

    rooms = [dict(anchor_i=ai, anchor_j=aj, cells=room_cells,
                  center_i=ai + (room_cells - 1) / 2.0,
                  center_j=aj + (room_cells - 1) / 2.0)
             for (ai, aj) in chosen]
    return vwalls, hwalls, rooms


def box(cx, cy, sx, sy, h, idx):
    z = h / 2.0
    return (
        f'      <collision name="c{idx}"><pose>{cx:.4f} {cy:.4f} {z:.4f} 0 0 0</pose>'
        f"<geometry><box><size>{sx:.4f} {sy:.4f} {h:.4f}</size></box></geometry></collision>\n"
        f'      <visual name="v{idx}"><pose>{cx:.4f} {cy:.4f} {z:.4f} 0 0 0</pose>'
        f"<geometry><box><size>{sx:.4f} {sy:.4f} {h:.4f}</size></box></geometry>"
        "<material><ambient>0.5 0.5 0.55 1</ambient><diffuse>0.6 0.6 0.65 1</diffuse></material>"
        "</visual>\n"
    )


def build_world(width, height, corridor, wall_nominal, wall_h, seed, origin,
                rooms=0, room_cells=3, room_min_sep_cells=6):
    nx, tx = solve_grid(width, corridor, wall_nominal)
    ny, ty = solve_grid(height, corridor, wall_nominal)
    px, py = corridor + tx, corridor + ty  # cell pitch

    def x_line(k):  # vertical wall centerline
        return k * px + tx / 2.0

    def y_line(k):  # horizontal wall centerline
        return k * py + ty / 2.0

    def x_cell(i):  # corridor center
        return i * px + tx + corridor / 2.0

    def y_cell(j):
        return j * py + ty + corridor / 2.0

    vwalls, hwalls = carve(nx, ny, seed)
    vwalls, hwalls, room_list = carve_rooms(vwalls, hwalls, nx, ny, rooms,
                                            room_cells, room_min_sep_cells, seed)

    # origin "center" shifts the whole maze so its center sits on the world origin
    # (0,0) -> maze spans [-w/2, w/2] x [-h/2, h/2], convenient for the Gazebo view.
    # origin "corner" keeps the bottom-left corner at (0,0).
    ox = -width / 2.0 if origin == "center" else 0.0
    oy = -height / 2.0 if origin == "center" else 0.0

    boxes = []
    numeric = []  # world-frame (cx, cy, sx, sy) for rasterizing the occupancy map

    def add(cx, cy, sx, sy):
        boxes.append(box(cx + ox, cy + oy, sx, sy, wall_h, len(boxes)))
        numeric.append((cx + ox, cy + oy, sx, sy))

    # perimeter (exact width x height)
    add(width / 2, ty / 2, width, ty)              # bottom
    add(width / 2, height - ty / 2, width, ty)     # top
    add(tx / 2, height / 2, tx, height)            # left
    add(width - tx / 2, height / 2, tx, height)    # right
    # interior vertical walls (span a full cell pitch in y to close corners)
    for (k, j) in sorted(vwalls):
        add(x_line(k), y_cell(j), tx, py)
    # interior horizontal walls
    for (i, k) in sorted(hwalls):
        add(x_cell(i), y_line(k), px, ty)

    spawn = (x_cell(0) + ox, y_cell(0) + oy)  # bottom-left free cell for the robot

    room_size_m = room_cells * px - tx if room_list else 0.0
    rooms_world = [(x_cell(r["center_i"]) + ox, y_cell(r["center_j"]) + oy)
                  for r in room_list]
    rooms_comment = ""
    if rooms_world:
        pts = "; ".join(f"({x:.2f},{y:.2f})" for x, y in rooms_world)
        rooms_comment = (f"\n     {len(rooms_world)} open rooms, {room_size_m:.2f} m square, "
                         f"centers: {pts}.")

    world = f"""<?xml version="1.0"?>
<!-- Maze world: {width} x {height} m, corridor {corridor} m ({corridor/0.30:.1f}x robot width),
     walls {tx:.3f} m (x) / {ty:.3f} m (y), grid {nx} x {ny} cells, seed {seed}, origin={origin}.
     Generated by generate_maze.py (edit the generator, not this file).
     Suggested robot spawn: x={spawn[0]:.2f} y={spawn[1]:.2f}.{rooms_comment} -->
<sdf version="1.9">
  <world name="maze">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="gz-sim-physics-system"           name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system"     name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system"           name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system"               name="gz::sim::systems::Imu"/>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>60 60</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>60 60</size></plane></geometry>
          <material><ambient>0.3 0.3 0.3 1</ambient><diffuse>0.5 0.5 0.5 1</diffuse></material>
        </visual>
      </link>
    </model>

    <model name="maze">
      <static>true</static>
      <link name="walls">
{''.join(boxes)}      </link>
    </model>
  </world>
</sdf>
"""
    meta = dict(nx=nx, ny=ny, tx=tx, ty=ty, boxes=len(boxes), spawn=spawn,
                numeric=numeric, ox=ox, oy=oy, rooms=rooms_world, room_size_m=room_size_m)
    return world, meta


def rasterize(numeric, width, height, ox, oy, res):
    """Rasterize wall boxes into a 2D occupancy image (row 0 = top = max y, the
    ROS map_server convention). Returns a list[rows] of list[cols] of 0/100.

    Fills per-box (only the cells each wall covers) rather than testing every cell
    against every box -> fast even at 0.05 m resolution.
    """
    cols = int(round(width / res))
    rows = int(round(height / res))
    img = [[0] * cols for _ in range(rows)]  # 0 = free

    def x_of(c):  # world x at column-center c
        return ox + (c + 0.5) * res

    def y_of(i):  # world y at image-row i (row 0 is top -> max y)
        return oy + (rows - i - 0.5) * res

    for (bcx, bcy, sx, sy) in numeric:
        hx, hy = sx / 2.0, sy / 2.0
        c0 = max(0, int((bcx - hx - ox) / res))
        c1 = min(cols - 1, int((bcx + hx - ox) / res))
        # higher y -> smaller image row, so top edge (bcy+hy) gives the min row
        i0 = max(0, int((oy + rows * res - (bcy + hy)) / res) - 1)
        i1 = min(rows - 1, int((oy + rows * res - (bcy - hy)) / res) + 1)
        for i in range(i0, i1 + 1):
            y = y_of(i)
            if not (bcy - hy <= y <= bcy + hy):
                continue
            for c in range(c0, c1 + 1):
                if bcx - hx <= x_of(c) <= bcx + hx:
                    img[i][c] = 100  # occupied
    return img


def write_map(map_out, img, res, ox, oy):
    """Write a ROS map_server pair: <map_out>.pgm (P5) + <map_out>.yaml.

    PGM values: 0 = occupied (black), 254 = free (white). With negate=0 and the
    default thresholds, map_server reads white as free and black as occupied.
    """
    base = map_out[:-5] if map_out.endswith(".yaml") else map_out
    pgm_path, yaml_path = base + ".pgm", base + ".yaml"
    rows, cols = len(img), len(img[0])

    header = f"P5\n{cols} {rows}\n255\n".encode("ascii")
    pixels = bytearray(rows * cols)
    for i in range(rows):
        row = img[i]
        off = i * cols
        for c in range(cols):
            pixels[off + c] = 0 if row[c] >= 50 else 254
    with open(pgm_path, "wb") as f:
        f.write(header)
        f.write(pixels)

    import os
    yaml = (
        f"image: {os.path.basename(pgm_path)}\n"
        f"mode: trinary\n"
        f"resolution: {res}\n"
        f"origin: [{ox:.4f}, {oy:.4f}, 0.0]\n"
        f"negate: 0\n"
        f"occupied_thresh: 0.65\n"
        f"free_thresh: 0.25\n"
    )
    with open(yaml_path, "w") as f:
        f.write(yaml)
    return pgm_path, yaml_path, rows, cols


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--width", type=float, default=20.0)
    ap.add_argument("--height", type=float, default=30.0)
    ap.add_argument("--corridor", type=float, default=1.5)   # 5 x 0.30 m robot width
    ap.add_argument("--wall", type=float, default=0.2)       # nominal (solved to exact)
    ap.add_argument("--wall-height", type=float, default=0.6)
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--origin", choices=["center", "corner"], default="center",
                    help="center: maze centered on world origin; corner: bottom-left at (0,0)")
    ap.add_argument("--out", default="maze.sdf")
    ap.add_argument("--map-out", default=None,
                    help="also emit a ROS occupancy map here (e.g. ../maps/maze.yaml)")
    ap.add_argument("--map-res", type=float, default=0.05,
                    help="occupancy map resolution in m/cell (default 0.05)")
    ap.add_argument("--rooms", type=int, default=0,
                    help="number of open (wall-free) rooms to carve, spread out by "
                         "--room-min-sep-cells (default 0 = original maze, unchanged)")
    ap.add_argument("--room-cells", type=int, default=3,
                    help="each room is this many grid cells square (default 3 -> "
                         "~4.5 m at the default 1.5 m corridor, comfortably larger "
                         "than any turning-radius maneuver needs)")
    ap.add_argument("--room-min-sep-cells", type=int, default=6,
                    help="minimum grid (hop-count, not straight-line) distance "
                         "between any two chosen rooms")
    a = ap.parse_args()

    world, meta = build_world(a.width, a.height, a.corridor, a.wall, a.wall_height,
                              a.seed, a.origin, a.rooms, a.room_cells,
                              a.room_min_sep_cells)
    with open(a.out, "w") as f:
        f.write(world)
    print(f"wrote {a.out}: grid {meta['nx']}x{meta['ny']}, "
          f"walls tx={meta['tx']:.3f} ty={meta['ty']:.3f}, "
          f"{meta['boxes']} boxes, spawn~({meta['spawn'][0]:.2f},{meta['spawn'][1]:.2f})")
    if meta["rooms"]:
        pts = ", ".join(f"({x:.2f},{y:.2f})" for x, y in meta["rooms"])
        print(f"  {len(meta['rooms'])} rooms, {meta['room_size_m']:.2f} m square, "
              f"centers: {pts}")
        if len(meta["rooms"]) < a.rooms:
            print(f"  WARNING: only found {len(meta['rooms'])}/{a.rooms} rooms "
                  f"satisfying --room-min-sep-cells={a.room_min_sep_cells} on this "
                  f"grid -- lower the separation or the room count")

    if a.map_out:
        img = rasterize(meta["numeric"], a.width, a.height,
                        meta["ox"], meta["oy"], a.map_res)
        pgm, yaml, rows, cols = write_map(a.map_out, img, a.map_res,
                                          meta["ox"], meta["oy"])
        print(f"wrote {pgm} + {yaml}: {cols}x{rows} cells @ {a.map_res} m, "
              f"origin=({meta['ox']:.2f},{meta['oy']:.2f})")


if __name__ == "__main__":
    main()
