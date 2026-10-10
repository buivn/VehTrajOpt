"""Host tests for the MPPI core (numpy only, no ROS/CasADi):

  python3 test/test_mppi_core.py

Closed-loop simulates the unicycle tracking a reference with MPPI and checks it
converges, stays near the path, is forward-only, and respects the limits.
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from vto_control.mppi_core import MPPICore            # noqa: E402
from vto_control.pursuit_core import sample_reference  # noqa: E402

V_MAX, W_MAX = 0.5, 1.5


def straight(length=4.0, ds=0.05):
    return [(x, 0.0) for x in np.arange(0.0, length + ds, ds)]


def arc(radius=2.0, da=0.03):
    return [(radius * math.sin(t), radius - radius * math.cos(t))
            for t in np.arange(0.0, math.pi / 2 + da, da)]


def simulate(mppi, path, start, max_steps=500):
    state = np.array(start, float)
    goal = np.array(path[-1])
    ds = 0.5 * mppi.dt
    max_cte = max_v = 0.0
    min_v = 1e9
    for _ in range(max_steps):
        if np.hypot(*(goal - state[:2])) < 0.15:
            break
        ref = np.array(sample_reference(path, state[0], state[1], mppi.N + 1, ds))
        v, w = mppi.control(state, ref)
        max_v, min_v = max(max_v, v), min(min_v, v)
        state = state + mppi.dt * np.array(
            [v * math.cos(state[2]), v * math.sin(state[2]), w])
        max_cte = max(max_cte, min(np.hypot(px - state[0], py - state[1])
                                   for px, py in path))
    reached = np.hypot(*(goal - state[:2])) < 0.15
    return reached, max_cte, max_v, min_v


def test_single_control_valid():
    mppi = MPPICore(seed=0)
    ref = np.array(sample_reference(straight(), 0.0, 0.0, mppi.N + 1, 0.05))
    v, w = mppi.control([0.0, 0.0, 0.0], ref)
    assert 0.0 <= v <= V_MAX + 1e-9 and abs(w) <= W_MAX + 1e-9
    print(f"PASS test_single_control_valid (v={v:.2f}, w={w:.2f})")


def test_track_straight():
    mppi = MPPICore(seed=1)
    reached, cte, mv, minv = simulate(mppi, straight(), [0.0, 0.0, 0.0])
    assert reached, "should reach end of straight"
    assert cte < 0.15, f"tracking too loose: {cte:.3f}"
    assert minv >= -1e-9, "forward-only: v must never go negative"
    assert mv <= V_MAX + 1e-3
    print(f"PASS test_track_straight (cte={cte:.3f} m, v in [{minv:.2f},{mv:.2f}])")


def test_track_arc():
    mppi = MPPICore(seed=2)
    reached, cte, mv, minv = simulate(mppi, arc(), [0.0, 0.0, 0.0])
    assert reached, "should reach end of arc"
    assert cte < 0.35, f"tracking too loose on arc: {cte:.3f}"
    assert minv >= -1e-9 and mv <= V_MAX + 1e-3
    print(f"PASS test_track_arc (cte={cte:.3f} m, v in [{minv:.2f},{mv:.2f}])")


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for t in tests:
        t()
    print(f"\nAll {len(tests)} tests passed.")
