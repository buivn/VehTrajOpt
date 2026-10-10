"""Tests the async solve / replay-last-plan state machine in mpc_controller_node.py
(docs/28), WITHOUT depending on real acados solve timing -- a FakeMPC with a
threading.Event stands in for MPCCoreAcados so every tick's outcome is deterministic.

Run in the container (needs rclpy + a real MPCCoreAcados just to construct the node):
  docker compose run --rm vto bash -lc \
    'source /opt/ros/jazzy/setup.bash && PYTHONPATH=/workspace/ros2_ws/src/vto_control \
     python3 /workspace/ros2_ws/src/vto_control/test/test_mpc_controller_async.py'
"""
import os
import sys
import threading

import numpy as np
import rclpy

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from vto_control.mpc_controller_node import MPCController  # noqa: E402


class FakeMPC:
    """solve() blocks until .gate is set, so the test controls exactly when each
    solve "finishes". Each solve's (2, N) sequence encodes (call number, column
    index) in its two rows, so replay order is directly readable off what gets
    published."""

    def __init__(self, N):
        self.N, self.dt = N, 0.1
        self.U_seq = np.zeros((2, N))
        self.gate = threading.Event()
        self.calls = 0

    def solve(self, state, u_prev, ref):
        self.gate.wait()
        self.gate.clear()
        self.calls += 1
        self.U_seq = np.array([[float(self.calls)] * self.N,
                                [float(k) for k in range(self.N)]])
        return float(self.U_seq[0, 0]), float(self.U_seq[1, 0])


def make_node():
    node = MPCController()
    node.mpc = FakeMPC(N=4)               # discard the real acados solver
    node._robot_pose_in_map = lambda: (0.0, 0.0, 0.0)
    node.path_pts = [(x, 0.0) for x in np.arange(0.0, 20.0, 0.1)]  # never "reached"
    node.reached = False
    published = []
    node._publish = lambda v, w: published.append((v, w))
    return node, published


def release_and_sync(node):
    """Set the gate and block THIS thread until the worker actually finishes,
    so the next _control() call deterministically sees future.done() == True."""
    node.mpc.gate.set()
    node._future.result()


def test_replay_sequence_and_exhaustion():
    node, pub = make_node()
    try:
        node._control()                          # tick 1: nothing to harvest, submits call #1
        assert pub == [], "should publish nothing before any solve finishes"
        assert node._future is not None and not node._future.done()

        node._control()                          # tick 2: call #1 still in flight, no plan yet
        assert pub[-1] == (0.0, 0.0), "no completed plan yet -> must stop, not guess"

        release_and_sync(node)
        node._control()                          # tick 3: harvest call #1, col 0
        assert pub[-1] == (1.0, 0.0)
        assert node._future is not None and not node._future.done(), \
            "call #2 should already be pipelined"

        node._control()                          # tick 4: call #2 in flight -> replay col 1
        assert pub[-1] == (1.0, 1.0), "should replay the 2nd control of the last plan"

        node._control()                          # tick 5: replay col 2
        assert pub[-1] == (1.0, 2.0), "should replay the 3rd control of the last plan"

        node._control()                          # tick 6: replay col 3 (last column)
        assert pub[-1] == (1.0, 3.0)

        node._control()                          # tick 7: plan exhausted -> hard stop
        assert pub[-1] == (0.0, 0.0), "must stop once the whole horizon is replayed"

        release_and_sync(node)
        node._control()                          # tick 8: harvest call #2, col 0
        assert pub[-1] == (2.0, 0.0)

        print("PASS test_replay_sequence_and_exhaustion")
    finally:
        # tick 8's step 2 pipelined call #3, which is now blocked on the gate --
        # release it so its worker thread exits before shutdown (ThreadPoolExecutor
        # joins workers at interpreter exit; a permanently-blocked one hangs it).
        node.mpc.gate.set()
        node.destroy_node()


def test_new_path_discards_stale_plan():
    node, pub = make_node()
    try:
        node._control()
        release_and_sync(node)
        node._control()                          # adopts call #1's plan
        assert node._seq is not None

        node._on_path(type("M", (), {"poses": []})())  # new (empty) path arrives
        assert node._future is None and node._seq is None, \
            "a new path must discard any plan computed for the old one"
        print("PASS test_new_path_discards_stale_plan")
    finally:
        # the node's _control() above also pipelined call #2 before _on_path ran;
        # it's now orphaned and blocked on the gate -- release it (see comment in
        # the other test's finally) so the worker thread can exit cleanly.
        node.mpc.gate.set()
        node.destroy_node()


if __name__ == "__main__":
    rclpy.init()
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for t in tests:
        t()
    rclpy.shutdown()
    print(f"\nAll {len(tests)} tests passed.")
