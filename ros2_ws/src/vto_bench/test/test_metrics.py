"""Host tests for the pure benchmark metrics (no ROS)."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from vto_bench.metrics import cross_track, path_length, summarize  # noqa: E402


def test_cross_track():
    path = [(0.0, 0.0), (10.0, 0.0)]        # a line along +x
    assert abs(cross_track(5.0, 0.0, path) - 0.0) < 1e-9    # on the line
    assert abs(cross_track(5.0, 0.3, path) - 0.3) < 1e-9    # 0.3 m off
    assert abs(cross_track(-1.0, 0.0, path) - 1.0) < 1e-9   # clamped past the start
    print("PASS test_cross_track")


def test_path_length():
    assert abs(path_length([(0, 0), (3, 0), (3, 4)]) - 7.0) < 1e-9
    print("PASS test_path_length")


def test_summarize():
    # 3 ticks moving +x at 0.5 m/s, tiny cte, steady w=0
    recs = [
        {"t": 0.0, "x": 0.0, "y": 0.00, "v": 0.5, "w": 0.0, "cte": 0.00,
         "clearance": 0.7, "compute_ms": 9.0},
        {"t": 0.1, "x": 0.05, "y": 0.02, "v": 0.5, "w": 0.1, "cte": 0.02,
         "clearance": 0.6, "compute_ms": 10.0},
        {"t": 0.2, "x": 0.10, "y": 0.00, "v": 0.5, "w": 0.0, "cte": 0.00,
         "clearance": 0.5, "compute_ms": 11.0},
    ]
    s = summarize(recs, planned_length=0.10, reached=True)
    assert s["reached"] == 1 and s["ticks"] == 3
    assert abs(s["time_to_goal"] - 0.2) < 1e-9
    assert abs(s["cte_max"] - 0.02) < 1e-9
    assert abs(s["min_clearance"] - 0.5) < 1e-9
    assert abs(s["compute_ms_mean"] - 10.0) < 1e-9
    assert s["dw_rms"] > 0.0            # w changed, so nonzero jerk proxy
    print(f"PASS test_summarize (cte_max={s['cte_max']}, dw_rms={s['dw_rms']:.3f})")


def test_summarize_empty():
    s = summarize([], planned_length=5.0, reached=False)
    assert s["reached"] == 0 and s["ticks"] == 0 and s["planned_len"] == 5.0
    print("PASS test_summarize_empty")


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for t in tests:
        t()
    print(f"\nAll {len(tests)} tests passed.")
