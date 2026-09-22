"""Reeds-Shepp curves -- bounded-curvature connections for a car-like
(Ackermann) state lattice that ALSO allow reverse motion. NO ROS deps.

Extends dubins.py (docs/24) exactly the way its own docstring predicted:
same edge-primitive role, same shape of interface (a Path object +
shortest_path() + sample()), but a segment may now be driven BACKWARD -- a
"cusp" is where direction reverses. This is what fixes cases Dubins
structurally cannot: reorienting in a pocket too tight to complete a
forward-only loop (docs/40's heading-9 trap -- PROVEN there by exhausting
the reachable state space: 877 states, none escaping, purely because
reverse wasn't an option).

Scope (deliberate, not the full 48-word Reeds & Shepp 1990 classification):
  - CSC family (8 words: same-turn / opposite-turn, times reflect (L<->R)
    and timeflip (fully-backward) symmetries) -- closed form, the same
    formulas' shape as dubins.py's CSC words, standard and well-verified.
  - CCC family, INCLUDING the reversed-middle-segment word ("C|C|C", e.g.
    L+R-L+ -- literally a K-turn / 3-point-turn) -- this is the one that
    actually supplies the new capability. Its closed form is intricate
    enough that re-deriving it from memory carries real correctness risk,
    so it's solved with a short, bounded Newton iteration on the exact
    forward-kinematics equations instead of a memorized trig identity --
    still O(1) per query (a handful of iterations), just not closed-form.
  - The plain-forward Dubins words (via dubins.py) are also offered as
    candidates, so a Reeds-Shepp query is NEVER worse than the Dubins
    answer for the same pose pair.

Not implemented: CCCC, CCSC, CCSCC (the remaining literature families).
These shave a few percent off path length in narrow angular ranges but add
no new REACHABILITY -- CSC+CCC already provably connects any two poses
(same completeness argument as parallel parking: small enough CCC
maneuvers achieve arbitrary relative reorientation), which is what's
actually needed here (eliminate "no path" cases), not global optimality.

Correctness safety net: every candidate -- closed-form or numeric -- is
verified by forward-simulating it and checking the endpoint actually lands
on `goal` (position + heading, within tolerance) before being offered to
the caller. A formula bug degrades to "this word quietly doesn't fire",
never a wrong, possibly-colliding path entering the lattice.

Units / convention: identical to dubins.py -- unit-agnostic (radius in
whatever units start/goal use), yaw in radians (atan2(dy,dx) convention).
"""
import math

from vto_planning.dubins import dubins_shortest_path, mod2pi

TWO_PI = 2.0 * math.pi
_ENDPOINT_TOL = 1e-6          # position+heading match tolerance (radius units)


def _wrap_pi(theta):
    """Wrap an angle to (-pi, pi]."""
    return theta - TWO_PI * math.floor((theta + math.pi) / TWO_PI)


def _polar(x, y):
    return math.hypot(x, y), math.atan2(y, x)


def _arc_end(x, y, theta, kappa, signed_len):
    """Endpoint of a constant-curvature segment (radius-normalized, so
    curvature is +-1 or 0) starting at (x,y,theta), signed arc length
    `signed_len` (negative = driven backward). Same closed-form update
    `_sample_arc` in lattice.py already uses, generalized to a non-zero
    starting heading and an explicit curvature sign."""
    if abs(kappa) < 1e-12:
        return x + signed_len * math.cos(theta), y + signed_len * math.sin(theta), theta
    new_theta = theta + kappa * signed_len
    new_x = x + (math.sin(new_theta) - math.sin(theta)) / kappa
    new_y = y - (math.cos(new_theta) - math.cos(theta)) / kappa
    return new_x, new_y, new_theta


# ===================== CSC family (closed form) ========================= #

def _LpSpLp(x, y, phi):
    """L+S+L+: same-turn CSC, always solvable (external tangent between two
    equal circles always exists)."""
    u, t = _polar(x - math.sin(phi), y - 1.0 + math.cos(phi))
    v = mod2pi(phi - t)
    return [("L", mod2pi(t)), ("S", u), ("L", v)]


def _LpSpRp(x, y, phi):
    """L+S+R+: opposite-turn CSC, needs the circles far enough apart for an
    internal tangent (u1_sq >= 4) -- None if too close, same shape as
    dubins.py's _lsr needing p_sq >= 0."""
    xi = x + math.sin(phi)
    eta = y - 1.0 - math.cos(phi)
    u1_sq = xi * xi + eta * eta
    if u1_sq < 4.0:
        return None
    u = math.sqrt(u1_sq - 4.0)
    theta = math.atan2(2.0, u)
    t = mod2pi(math.atan2(eta, xi) + theta)
    v = mod2pi(t - phi)
    return [("L", t), ("S", u), ("R", v)]


_CSC_BASE = (_LpSpLp, _LpSpRp)


def _reflect_word(word):
    """Mirror across the local x-axis: L<->R, magnitudes unchanged."""
    swap = {"L": "R", "R": "L", "S": "S"}
    return [(swap[letter], val) for letter, val in word]


def _reverse_word(word):
    """Timeflip: reverse segment order, negate every magnitude (drive the
    whole path backward -- docs/41 module docstring derives why this,
    not a sign-preserving reversal, is the correct transform)."""
    return [(letter, -val) for letter, val in reversed(word)]


def _csc_candidates(x, y, phi, gx, gy, gphi):
    """All 8 CSC words: 2 base families x reflect x timeflip.
    (x,y,phi) = goal in start's local frame; (gx,gy,gphi) = start in goal's
    local frame (needed for the timeflip half, which solves the swapped
    query then reverses it -- see module docstring)."""
    out = []
    for fn in _CSC_BASE:
        w = fn(x, y, phi)
        if w is not None:
            out.append(w)
        w_refl = fn(x, -y, -phi)
        if w_refl is not None:
            out.append(_reflect_word(w_refl))
        w_tf = fn(gx, gy, gphi)
        if w_tf is not None:
            out.append(_reverse_word(w_tf))
        w_tf_refl = fn(gx, -gy, -gphi)
        if w_tf_refl is not None:
            out.append(_reverse_word(_reflect_word(w_tf_refl)))
    return out


# ===================== CCC family (reversed middle, numeric) ============ #

def _ccc_compose(t, u, v, letters):
    """Forward-simulate an (L/R, L/R, L/R) triple with signed segment
    lengths (t, -u, v) -- the middle segment backward, magnitude u >= 0 --
    from local start (0,0,0). `letters` is e.g. ("L","R","L")."""
    kappas = [1.0 if L == "L" else -1.0 for L in letters]
    x, y, th = 0.0, 0.0, 0.0
    x, y, th = _arc_end(x, y, th, kappas[0], t)
    x, y, th = _arc_end(x, y, th, kappas[1], -u)
    x, y, th = _arc_end(x, y, th, kappas[2], v)
    return x, y, th


def _ccc_residual(params, x, y, phi, letters):
    t, u, v = params
    px, py, pth = _ccc_compose(t, u, v, letters)
    return (px - x, py - y, _wrap_pi(pth - phi))


def _ccc_solve(x, y, phi, letters, seeds):
    """Bounded Newton solve for the reversed-middle CCC family: 3 equations
    (x, y, heading match), 3 unknowns (t, u, v >= 0), numeric Jacobian.
    Tried from several seeds since the system isn't globally convex; keeps
    the best-converged, geometrically valid (all >= 0) result."""
    best = None
    for seed in seeds:
        p = list(seed)
        for _ in range(30):
            r0 = _ccc_residual(p, x, y, phi, letters)
            err = math.sqrt(sum(c * c for c in r0))
            if err < 1e-10:
                break
            J = [[0.0] * 3 for _ in range(3)]
            h = 1e-6
            for j in range(3):
                pj = list(p)
                pj[j] += h
                rj = _ccc_residual(pj, x, y, phi, letters)
                for i in range(3):
                    J[i][j] = (rj[i] - r0[i]) / h
            delta = _solve3(J, [-c for c in r0])
            if delta is None:
                break
            p = [p[k] + delta[k] for k in range(3)]
        r_final = _ccc_residual(p, x, y, phi, letters)
        err = math.sqrt(sum(c * c for c in r_final))
        t, u, v = p
        if err < 1e-7 and t >= -1e-9 and u >= -1e-9 and v >= -1e-9:
            cost = abs(t) + abs(u) + abs(v)
            if best is None or cost < best[0]:
                best = (cost, max(t, 0.0), max(u, 0.0), max(v, 0.0))
    if best is None:
        return None
    _, t, u, v = best
    return [(letters[0], t), (letters[1], -u), (letters[2], v)]


def _solve3(J, b):
    """Plain 3x3 linear solve via Cramer's rule -- no numpy dependency."""
    def det3(m):
        return (m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
                - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
                + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]))
    d = det3(J)
    if abs(d) < 1e-14:
        return None
    x = [0.0, 0.0, 0.0]
    for i in range(3):
        Jc = [row[:] for row in J]
        for r in range(3):
            Jc[r][i] = b[r]
        x[i] = det3(Jc) / d
    return x


_CCC_SEEDS = [(0.5, 1.0, 0.5), (1.0, 2.0, 1.0), (2.0, 1.0, 2.0),
             (0.2, 3.0, 0.2), (1.5, 0.5, 1.5)]


def _ccc_candidates(x, y, phi, gx, gy, gphi):
    """The reversed-middle-segment words: L+R-L+, R+L-R+ (reflect), and
    their timeflips (backward-first/forward-middle/backward-last)."""
    out = []
    for letters in (("L", "R", "L"), ("R", "L", "R")):
        w = _ccc_solve(x, y, phi, letters, _CCC_SEEDS)
        if w is not None:
            out.append(w)
        w_tf = _ccc_solve(gx, gy, gphi, letters, _CCC_SEEDS)
        if w_tf is not None:
            out.append(_reverse_word(w_tf))
    return out


# ===================== public interface ================================= #

class ReedsSheppPath:
    """One solved Reeds-Shepp curve. `segments` is a list of
    (letter, signed_value): letter in {"L","R","S"}; signed_value is a
    turn angle (radians) for L/R or a distance for S, in NORMALIZED
    (radius=1) units -- negative means that segment is driven BACKWARD.
    `length` is the real total arc length (sum of |signed_value| * radius,
    consistent with a real car's odometer either way)."""

    __slots__ = ("start", "goal", "radius", "segments", "length")

    def __init__(self, start, goal, radius, segments):
        self.start = start
        self.goal = goal
        self.radius = radius
        self.segments = segments
        self.length = sum(abs(v) for _, v in segments) * radius

    def __repr__(self):
        segs = " ".join(f"{l}{v:+.3f}" for l, v in self.segments)
        return f"ReedsSheppPath([{segs}], len={self.length:.4f})"


def _verify(segments, x, y, phi):
    """Forward-simulate `segments` from local (0,0,0) and check it lands on
    (x,y,phi) -- the safety net the module docstring promises."""
    px, py, pth = 0.0, 0.0, 0.0
    for letter, val in segments:
        kappa = {"L": 1.0, "R": -1.0, "S": 0.0}[letter]
        px, py, pth = _arc_end(px, py, pth, kappa, val)
    return (abs(px - x) < _ENDPOINT_TOL and abs(py - y) < _ENDPOINT_TOL
            and abs(_wrap_pi(pth - phi)) < _ENDPOINT_TOL)


def reeds_shepp_shortest_path(start, goal, radius):
    """Shortest bounded-curvature curve from `start` to `goal` ALLOWING
    reverse motion. start, goal: (x, y, yaw). radius: minimum turning
    radius (> 0). Returns a ReedsSheppPath, or raises RuntimeError if no
    candidate word (closed-form or numeric) verifies -- practically this
    should not happen (Reeds & Shepp's theorem guarantees a solution
    always exists), but this module doesn't implement the full 48-word
    catalog, so a caller in a genuinely exotic configuration could hit it;
    treat it as "widen the search" the same way dubins.py's RuntimeError
    is treated as a numerical-pathology signal, not a routine outcome.
    """
    if radius <= 0.0:
        raise ValueError("radius must be positive")
    sx, sy, syaw = start
    gx, gy, gyaw = goal

    def local(frame_origin, frame_yaw, px, py, pyaw):
        dx, dy = px - frame_origin[0], py - frame_origin[1]
        cos_f, sin_f = math.cos(frame_yaw), math.sin(frame_yaw)
        lx = (cos_f * dx + sin_f * dy) / radius
        ly = (-sin_f * dx + cos_f * dy) / radius
        lphi = mod2pi(pyaw - frame_yaw)
        return lx, ly, lphi

    x, y, phi = local((sx, sy), syaw, gx, gy, gyaw)          # goal in start's frame
    gx2, gy2, gphi = local((gx, gy), gyaw, sx, sy, syaw)     # start in goal's frame

    candidates = []
    candidates.extend(_csc_candidates(x, y, phi, gx2, gy2, gphi))
    candidates.extend(_ccc_candidates(x, y, phi, gx2, gy2, gphi))
    # plain-forward Dubins words too, so Reeds-Shepp is never worse:
    dpath = dubins_shortest_path(start, goal, radius)
    candidates.append([(letter, seg) for letter, seg in zip(dpath.modes, dpath.seg_norm)])

    best = None
    for word in candidates:
        if not _verify(word, x, y, phi):
            continue
        cost = sum(abs(v) for _, v in word)
        if best is None or cost < best[0]:
            best = (cost, word)

    if best is None:
        raise RuntimeError(f"no Reeds-Shepp word verified for "
                           f"x={x:.4f} y={y:.4f} phi={phi:.4f}")
    return ReedsSheppPath(start, goal, radius, best[1])


def reeds_shepp_path_length(start, goal, radius):
    """Just the length of the shortest Reeds-Shepp curve -- an admissible
    non-holonomic heuristic when reverse is allowed (dubins_path_length's
    counterpart)."""
    return reeds_shepp_shortest_path(start, goal, radius).length


def sample_reeds_shepp(path, step):
    """Poses (x, y, yaw) along `path`, spaced at most `step` apart (real
    units), first and last included. Reuses the exact per-segment
    constant-curvature update dubins.py's sample_dubins uses -- the only
    difference is a segment's signed length can be negative (backward),
    which the same closed form already handles correctly (docs/41 module
    docstring: this is why sampling needed no new integration math, only
    sign-awareness)."""
    pts = [path.start]
    x, y, yaw = path.start
    for letter, val in path.segments:
        seg_len = val * path.radius
        if abs(seg_len) < 1e-12:
            continue
        n = max(1, int(math.ceil(abs(seg_len) / step)))
        kappa = {"L": 1.0 / path.radius, "R": -1.0 / path.radius, "S": 0.0}[letter]
        for i in range(1, n + 1):
            frac = i / n
            px, py, pyaw = _arc_end(x, y, yaw, kappa, seg_len * frac)
            pts.append((px, py, pyaw))
        x, y, yaw = pts[-1]
    return pts
