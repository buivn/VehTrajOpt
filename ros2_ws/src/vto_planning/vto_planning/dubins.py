"""Dubins shortest-path curves — bounded-curvature connections for a car-like
(Ackermann) state lattice. NO ROS dependencies.

See docs/24 for the full design. This is step 1 of the Ackermann-lattice build:
the *edge* primitive. Given two oriented poses (x, y, yaw) and a minimum turning
radius, it returns the shortest FORWARD-ONLY curve a bounded-curvature car can
drive between them — always a sequence of at most three segments drawn from
{L (left arc), R (right arc), S (straight)}, one of six "words":

    CSC:  LSL  RSR  LSR  RSL
    CCC:  RLR  LRL

Each word has a closed-form solution for its three segment lengths; compute all
six, drop the infeasible ones, keep the shortest. O(1), no iteration.

Units: like astar.py / lattice.py this module is unit-agnostic. lattice_ackermann
(step 2) will call it in CELL units with radius = R_min / map_resolution.

Heading convention: yaw in radians, measured the usual way (atan2(dy, dx)).
Forward motion only (no reverse / cusps) — that is Reeds-Shepp, a documented
later drop-in behind the same interface.

Formulas follow the normalized (alpha, beta, d) treatment of
Shkel & Lumelsky, "Classification of the Dubins set" (2001).
"""
import math

TWO_PI = 2.0 * math.pi


def mod2pi(theta):
    """Wrap an angle to [0, 2*pi)."""
    return theta - TWO_PI * math.floor(theta / TWO_PI)


# --- the six words: (alpha, beta, d) -> (t, p, q) in normalized units, or None ---
# alpha, beta: start / goal heading relative to the chord direction.
# d:           straight-line start->goal distance, divided by the turning radius.
# t, p, q:     the three segment lengths (turns in radians, straight / radius).

def _lsl(alpha, beta, d):
    sa, sb = math.sin(alpha), math.sin(beta)
    ca, cb = math.cos(alpha), math.cos(beta)
    c_ab = math.cos(alpha - beta)
    p_sq = 2.0 + d * d - 2.0 * c_ab + 2.0 * d * (sa - sb)
    if p_sq < 0.0:
        return None
    tmp = math.atan2(cb - ca, d + sa - sb)
    return mod2pi(tmp - alpha), math.sqrt(p_sq), mod2pi(beta - tmp)


def _rsr(alpha, beta, d):
    sa, sb = math.sin(alpha), math.sin(beta)
    ca, cb = math.cos(alpha), math.cos(beta)
    c_ab = math.cos(alpha - beta)
    p_sq = 2.0 + d * d - 2.0 * c_ab + 2.0 * d * (sb - sa)
    if p_sq < 0.0:
        return None
    tmp = math.atan2(ca - cb, d - sa + sb)
    return mod2pi(alpha - tmp), math.sqrt(p_sq), mod2pi(tmp - beta)


def _lsr(alpha, beta, d):
    sa, sb = math.sin(alpha), math.sin(beta)
    ca, cb = math.cos(alpha), math.cos(beta)
    c_ab = math.cos(alpha - beta)
    p_sq = -2.0 + d * d + 2.0 * c_ab + 2.0 * d * (sa + sb)
    if p_sq < 0.0:
        return None
    p = math.sqrt(p_sq)
    tmp = math.atan2(-ca - cb, d + sa + sb) - math.atan2(-2.0, p)
    return mod2pi(tmp - alpha), p, mod2pi(tmp - mod2pi(beta))


def _rsl(alpha, beta, d):
    sa, sb = math.sin(alpha), math.sin(beta)
    ca, cb = math.cos(alpha), math.cos(beta)
    c_ab = math.cos(alpha - beta)
    p_sq = -2.0 + d * d + 2.0 * c_ab - 2.0 * d * (sa + sb)
    if p_sq < 0.0:
        return None
    p = math.sqrt(p_sq)
    tmp = math.atan2(ca + cb, d - sa - sb) - math.atan2(2.0, p)
    return mod2pi(alpha - tmp), p, mod2pi(mod2pi(beta) - tmp)


def _rlr(alpha, beta, d):
    sa, sb = math.sin(alpha), math.sin(beta)
    ca, cb = math.cos(alpha), math.cos(beta)
    c_ab = math.cos(alpha - beta)
    tmp = (6.0 - d * d + 2.0 * c_ab + 2.0 * d * (sa - sb)) / 8.0
    if abs(tmp) > 1.0:
        return None
    p = mod2pi(TWO_PI - math.acos(tmp))
    t = mod2pi(alpha - math.atan2(ca - cb, d - sa + sb) + p / 2.0)
    return t, p, mod2pi(alpha - beta - t + p)


def _lrl(alpha, beta, d):
    sa, sb = math.sin(alpha), math.sin(beta)
    ca, cb = math.cos(alpha), math.cos(beta)
    c_ab = math.cos(alpha - beta)
    tmp = (6.0 - d * d + 2.0 * c_ab + 2.0 * d * (sb - sa)) / 8.0
    if abs(tmp) > 1.0:
        return None
    p = mod2pi(TWO_PI - math.acos(tmp))
    t = mod2pi(-alpha + math.atan2(-ca + cb, d + sa - sb) + p / 2.0)
    return t, p, mod2pi(mod2pi(beta) - alpha - t + p)


_WORDS = (
    ("LSL", _lsl), ("RSR", _rsr), ("LSR", _lsr),
    ("RSL", _rsl), ("RLR", _rlr), ("LRL", _lrl),
)


class DubinsPath:
    """One solved Dubins curve. `word` is e.g. "LSR"; `seg_norm` the three
    segment lengths in normalized units (turns in radians, straight divided by
    radius). `length` is the real arc length = sum(seg_norm) * radius — the same
    `* radius` works for all three because turn length in radians times radius is
    arc length, and the straight term was divided by radius on the way in."""

    __slots__ = ("start", "goal", "radius", "word", "seg_norm", "length")

    def __init__(self, start, goal, radius, word, seg_norm):
        self.start = start
        self.goal = goal
        self.radius = radius
        self.word = word
        self.seg_norm = seg_norm
        self.length = sum(seg_norm) * radius

    @property
    def modes(self):
        return tuple(self.word)          # ("L", "S", "R")

    def __repr__(self):
        return (f"DubinsPath({self.word}, len={self.length:.4f}, "
                f"segs={tuple(round(s, 4) for s in self.seg_norm)})")


def dubins_shortest_path(start, goal, radius):
    """Shortest forward-only bounded-curvature curve from `start` to `goal`.

    start, goal: (x, y, yaw). radius: minimum turning radius (> 0).
    Returns a DubinsPath. Never None — Dubins' theorem guarantees a solution
    exists for any radius > 0; a RuntimeError is raised only if that invariant
    is somehow violated (numerical pathology), since callers rely on it.
    """
    if radius <= 0.0:
        raise ValueError("radius must be positive")
    sx, sy, syaw = start
    gx, gy, gyaw = goal

    # express the goal in a frame translated to `start` and rotated so the start
    # heading is 0 — this strips SE(2) down to the 3 intrinsic DOF (alpha, beta, d)
    dx, dy = gx - sx, gy - sy
    cos_s, sin_s = math.cos(syaw), math.sin(syaw)
    local_x = cos_s * dx + sin_s * dy
    local_y = -sin_s * dx + cos_s * dy

    dist = math.hypot(local_x, local_y)
    d = dist / radius
    theta = mod2pi(math.atan2(local_y, local_x))
    alpha = mod2pi(-theta)
    beta = mod2pi(mod2pi(gyaw - syaw) - theta)

    best = None
    for word, fn in _WORDS:
        sol = fn(alpha, beta, d)
        if sol is None:
            continue
        cost = sol[0] + sol[1] + sol[2]
        if best is None or cost < best[0]:
            best = (cost, word, sol)

    if best is None:
        raise RuntimeError(f"no Dubins word feasible for "
                           f"alpha={alpha:.4f} beta={beta:.4f} d={d:.4f}")
    _, word, seg_norm = best
    return DubinsPath(start, goal, radius, word, seg_norm)


def dubins_path_length(start, goal, radius):
    """Just the length of the shortest Dubins curve — handy as an admissible
    non-holonomic heuristic (Dubins distance ignoring obstacles)."""
    return dubins_shortest_path(start, goal, radius).length


def sample_dubins(path, step):
    """Poses (x, y, yaw) along `path`, spaced at most `step` apart (real units),
    first and last points included. Each segment is advanced by its exact
    constant-curvature update, so the final sample equals `path.goal` to
    machine precision."""
    pts = [path.start]
    x, y, yaw = path.start
    for mode, seg in zip(path.modes, path.seg_norm):
        seg_len = seg * path.radius              # real arc length of this segment
        if seg_len < 1e-12:
            continue
        n = max(1, int(math.ceil(seg_len / step)))
        for i in range(1, n + 1):
            u = seg * i / n                      # normalized progress into segment
            if mode == "S":
                px = x + u * path.radius * math.cos(yaw)
                py = y + u * path.radius * math.sin(yaw)
                pyaw = yaw
            elif mode == "L":
                px = x + path.radius * (math.sin(yaw + u) - math.sin(yaw))
                py = y - path.radius * (math.cos(yaw + u) - math.cos(yaw))
                pyaw = mod2pi(yaw + u)
            else:  # "R"
                px = x - path.radius * (math.sin(yaw - u) - math.sin(yaw))
                py = y + path.radius * (math.cos(yaw - u) - math.cos(yaw))
                pyaw = mod2pi(yaw - u)
            pts.append((px, py, pyaw))
        x, y, yaw = pts[-1]
    return pts
