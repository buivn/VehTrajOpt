"""MPPI (Model Predictive Path Integral) core for a unicycle robot — NO ROS.

Framework-free (like astar.py / pursuit_core.py / mpc_core.py): vectorised numpy,
host-testable. Sampling-based MPC (see docs/15): each step, sample K noisy control
sequences around a nominal, roll them all out through the unicycle, score them, and
set the command to the cost-weighted (path-integral) average.

    state x = (x, y, theta)      control u = (v, omega)
Rollouts are vectorised over the K samples; the only loop is over the N horizon steps.
Sampling is forward-only by default (v clipped to [v_min, v_max], v_min=0) — see the
AMCL-divergence lesson in docs/14.
"""
import math

import numpy as np


class MPPICore:
    def __init__(self, N=20, dt=0.1, K=1000, lambda_=1.0,
                 sigma_v=0.25, sigma_w=0.6,
                 v_min=0.0, v_max=0.5, w_max=1.5,
                 w_pos=12.0, w_theta=0.5, w_ctrl=0.02, seed=None):
        self.N, self.dt, self.K = N, dt, K
        self.lambda_ = lambda_
        self.sigma = np.array([sigma_v, sigma_w])
        self.v_min, self.v_max, self.w_max = v_min, v_max, w_max
        self.w_pos, self.w_theta, self.w_ctrl = w_pos, w_theta, w_ctrl
        self.rng = np.random.default_rng(seed)
        self.U = np.zeros((N, 2))          # nominal control sequence (warm-started)

    def reset(self):
        self.U[:] = 0.0

    def _rollout(self, state, V):
        """Vectorised unicycle rollout. V:(K,N,2) -> X:(K,N+1,3)."""
        K, N = V.shape[0], self.N
        X = np.empty((K, N + 1, 3))
        X[:, 0, :] = state
        for t in range(N):
            v, w = V[:, t, 0], V[:, t, 1]
            th = X[:, t, 2]
            X[:, t + 1, 0] = X[:, t, 0] + v * np.cos(th) * self.dt
            X[:, t + 1, 1] = X[:, t, 1] + v * np.sin(th) * self.dt
            X[:, t + 1, 2] = X[:, t, 2] + w * self.dt
        return X

    def control(self, state, ref):
        """One MPPI step.
        Args:
            state: (3,) current [x, y, theta] (map frame).
            ref:   (N+1, 3) reference states over the horizon.
        Returns:
            (v, omega) — the first control to apply.
        """
        state = np.asarray(state, float)
        ref = np.asarray(ref, float)
        N, K = self.N, self.K

        # 1) sample K noisy control sequences around the nominal, forward-only clip
        eps = self.rng.normal(0.0, 1.0, (K, N, 2)) * self.sigma
        V = self.U[None, :, :] + eps                       # (K,N,2)
        V[:, :, 0] = np.clip(V[:, :, 0], self.v_min, self.v_max)
        V[:, :, 1] = np.clip(V[:, :, 1], -self.w_max, self.w_max)

        # 2) roll out all K
        X = self._rollout(state, V)                        # (K,N+1,3)

        # 3) cost per rollout (tracking + heading + control effort)
        dx = X[:, :, 0] - ref[None, :, 0]
        dy = X[:, :, 1] - ref[None, :, 1]
        dth = X[:, :, 2] - ref[None, :, 2]
        dth = np.arctan2(np.sin(dth), np.cos(dth))         # wrap heading error
        S = (self.w_pos * (dx ** 2 + dy ** 2)).sum(axis=1)
        S += (self.w_theta * dth ** 2).sum(axis=1)
        S += (self.w_ctrl * (V[:, :, 0] ** 2 + V[:, :, 1] ** 2)).sum(axis=1)

        # 4) path-integral weights (softmax of -cost/lambda), then weighted average
        w = np.exp(-(S - S.min()) / self.lambda_)
        w /= w.sum()
        self.U = (w[:, None, None] * V).sum(axis=0)        # (N,2)

        u0 = self.U[0].copy()
        # 5) warm start: shift nominal forward (repeat last)
        self.U[:-1] = self.U[1:]
        return (float(u0[0]), float(u0[1]))
