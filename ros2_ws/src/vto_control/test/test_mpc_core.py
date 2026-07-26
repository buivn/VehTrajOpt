"""Tests for the MPC core (needs CasADi -> run in the container):

  docker run --rm -v $HOME/projects/2026/VehTrajOpt:/workspace vehtrajopt:jazzy \
    bash -lc 'python3 /workspace/ros2_ws/src/vto_control/test/test_mpc_core.py'

Closed-loop simulates the unicycle tracking a reference with the MPC and checks it
converges, stays on the path, and never exceeds the velocity limits.
"""
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from vto_control.mpc_core import MPCCore  # noqa: E402

V_MAX, W_MAX = 0.5, 1.5


def straight_path(length=4.0, ds=0.04):
    xs = np.arange(0.0, length + ds, ds)
    return np.array([[x, 0.0, 0.0] for x in xs])


def arc_path(radius=2.0, da=0.02):
    a = np.arange(0.0, np.pi / 2 + da, da)
    return np.array([[radius * np.sin(t), radius - radius * np.cos(t), t] for t in a])


def window(path, xy, N):
    """(3, N+1) reference window: nearest path point + N ahead (clamped at the end)."""
    d = np.hypot(path[:, 0] - xy[0], path[:, 1] - xy[1])
    i = int(np.argmin(d))
    idx = [min(i + k, len(path) - 1) for k in range(N + 1)]
    return path[idx].T


def simulate(mpc, path, start, max_steps=400):
    state = np.array(start, dtype=float)
    u_prev = np.array([0.0, 0.0])
    goal = path[-1, :2]
    max_err = max_v = max_w = 0.0
    solves, t0 = 0, time.time()
    for _ in range(max_steps):
        if np.hypot(*(goal - state[:2])) < 0.12:
            break
        ref = window(path, state[:2], mpc.N)
        v, w = mpc.solve(state, u_prev, ref)
        solves += 1
        max_v, max_w = max(max_v, abs(v)), max(max_w, abs(w))
        state = state + mpc.dt * np.array(
            [v * np.cos(state[2]), v * np.sin(state[2]), w])
        u_prev = np.array([v, w])
        max_err = max(max_err, float(np.hypot(path[:, 0] - state[0],
                                              path[:, 1] - state[1]).min()))
    reached = np.hypot(*(goal - state[:2])) < 0.12
    ms = 1000 * (time.time() - t0) / max(solves, 1)
    return reached, max_err, max_v, max_w, ms


def test_single_solve_valid():
    mpc = MPCCore()
    ref = window(straight_path(), [0.0, 0.0], mpc.N)
    v, w = mpc.solve([0.0, 0.0, 0.0], [0.0, 0.0], ref)
    assert abs(v) <= V_MAX + 1e-6 and abs(w) <= W_MAX + 1e-6
    assert v > 0.0, "should command forward motion toward the path ahead"
    print(f"PASS test_single_solve_valid (v={v:.2f}, w={w:.2f})")


def test_track_straight():
    mpc = MPCCore()
    reached, err, mv, mw, ms = simulate(mpc, straight_path(), [0.0, 0.0, 0.0])
    assert reached, "should reach the end of the straight line"
    assert err < 0.10, f"tracking error too high: {err:.3f}"
    assert mv <= V_MAX + 1e-3 and mw <= W_MAX + 1e-3, "velocity limits violated"
    print(f"PASS test_track_straight (err={err:.3f} m, max_v={mv:.2f}, {ms:.1f} ms/solve)")


def test_track_arc():
    mpc = MPCCore()
    reached, err, mv, mw, ms = simulate(mpc, arc_path(), [0.0, 0.0, 0.0])
    assert reached, "should reach the end of the arc"
    assert err < 0.20, f"tracking error too high on the arc: {err:.3f}"
    assert mv <= V_MAX + 1e-3 and mw <= W_MAX + 1e-3
    print(f"PASS test_track_arc (err={err:.3f} m, max_v={mv:.2f}, {ms:.1f} ms/solve)")


def test_recovers_from_heading_error():
    # start facing +y but the path goes +x -> MPC must turn to align, then track.
    mpc = MPCCore()
    reached, err, mv, mw, ms = simulate(mpc, straight_path(), [0.0, 0.0, np.pi / 2])
    assert reached, "should recover from a 90-deg initial heading error"
    assert mw <= W_MAX + 1e-3
    print(f"PASS test_recovers_from_heading_error (err={err:.3f} m, {ms:.1f} ms/solve)")


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for t in tests:
        t()
    print(f"\nAll {len(tests)} tests passed.")
