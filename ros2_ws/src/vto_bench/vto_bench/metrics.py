"""Pure metric helpers for the controller benchmark — NO ROS. Host-testable.

A "tick" is one sample: dict with keys t, x, y, v, w, cte, clearance, compute_ms.
`summarize()` turns a leg's ticks into the 7 report metrics.
"""
import math


def path_length(path):
    """Arc length of a polyline [(x, y), …]."""
    return sum(math.hypot(bx - ax, by - ay)
               for (ax, ay), (bx, by) in zip(path, path[1:]))


def cross_track(x, y, path):
    """Min distance from (x, y) to the path polyline (the tracking error)."""
    if not path:
        return 0.0
    if len(path) == 1:
        return math.hypot(x - path[0][0], y - path[0][1])
    best = float("inf")
    for (ax, ay), (bx, by) in zip(path, path[1:]):
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy or 1e-12
        t = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / L2))
        best = min(best, math.hypot(x - (ax + t * dx), y - (ay + t * dy)))
    return best


def _mean(a):
    return sum(a) / len(a) if a else 0.0


def _rms(a):
    return math.sqrt(sum(v * v for v in a) / len(a)) if a else 0.0


def summarize(records, planned_length, reached):
    """Compute the 7-metric summary for one leg.

    Returns a flat dict (one CSV row). Metric groups:
      accuracy  : cte_mean / cte_max / cte_rms                 (metric 1)
      efficiency: time_to_goal (2), driven_len/planned_len/len_ratio (3)
      comfort   : dw_rms / dv_rms  (4, jerk proxy), effort/mean_abs_w (5)
      safety    : min_clearance                                 (metric 6)
      compute   : compute_ms_mean / compute_ms_max              (metric 7)
    """
    n = len(records)
    cols = ["reached", "ticks", "time_to_goal",
            "cte_mean", "cte_max", "cte_rms",
            "driven_len", "planned_len", "len_ratio",
            "dw_rms", "dv_rms", "effort", "mean_abs_w", "mean_v",
            "min_clearance", "compute_ms_mean", "compute_ms_max"]
    if n == 0:
        d = {c: 0.0 for c in cols}
        d["reached"] = int(reached)
        d["planned_len"] = planned_length
        return d

    t = [r["t"] for r in records]
    x = [r["x"] for r in records]
    y = [r["y"] for r in records]
    v = [r["v"] for r in records]
    w = [r["w"] for r in records]
    cte = [r["cte"] for r in records]
    clr = [r["clearance"] for r in records]
    cms = [r["compute_ms"] for r in records if r.get("compute_ms") is not None]

    driven = sum(math.hypot(x[k + 1] - x[k], y[k + 1] - y[k]) for k in range(n - 1))
    dw = [w[k + 1] - w[k] for k in range(n - 1)]      # command-rate (jerk proxy)
    dv = [v[k + 1] - v[k] for k in range(n - 1)]

    return {
        "reached": int(reached),
        "ticks": n,
        "time_to_goal": (t[-1] - t[0]) if n > 1 else 0.0,
        "cte_mean": _mean(cte), "cte_max": max(cte), "cte_rms": _rms(cte),
        "driven_len": driven, "planned_len": planned_length,
        "len_ratio": (driven / planned_length) if planned_length > 0 else 0.0,
        "dw_rms": _rms(dw), "dv_rms": _rms(dv),
        "effort": _mean([vi * vi + wi * wi for vi, wi in zip(v, w)]),
        "mean_abs_w": _mean([abs(wi) for wi in w]), "mean_v": _mean(v),
        "min_clearance": min(clr) if clr else 0.0,
        "compute_ms_mean": _mean(cms), "compute_ms_max": max(cms) if cms else 0.0,
    }


SUMMARY_COLUMNS = ["controller", "repeat", "leg", "start_x", "start_y",
                   "goal_x", "goal_y", "reached", "ticks", "time_to_goal",
                   "cte_mean", "cte_max", "cte_rms", "driven_len", "planned_len",
                   "len_ratio", "dw_rms", "dv_rms", "effort", "mean_abs_w",
                   "mean_v", "min_clearance", "compute_ms_mean", "compute_ms_max"]
