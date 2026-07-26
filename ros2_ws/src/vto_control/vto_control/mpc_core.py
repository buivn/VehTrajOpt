"""Nonlinear MPC core for a differential-drive (unicycle) robot — NO ROS deps.

Framework-free (like pursuit_core.py / astar.py): builds the CasADi NLP ONCE and
re-solves it each tick with new parameter values. The ROS node is a thin adapter.

Formulation (see docs/13): multiple shooting, unicycle dynamics as equality
constraints, cost = tracking + effort + comfort + terminal.
    state  x = (x, y, theta)      control u = (v, omega)
    x_{k+1} = x_k + dt * (v cos th, v sin th, omega)
Decision vars: X (3 x N+1), U (2 x N). Parameters: current state x0, previous
control u_prev (anchors accel/comfort), and the N+1 reference states.
"""
import casadi as ca
import numpy as np


def _wrap(angle):
    """Wrap to (-pi, pi] symbolically (so heading error never blows up near ±pi)."""
    return ca.atan2(ca.sin(angle), ca.cos(angle))


class MPCCore:
    def __init__(self, N=20, dt=0.1,
                 v_max=0.5, w_max=1.5, dv_max=0.15, dw_max=0.30,
                 q_pos=10.0, q_theta=0.5,           # tracking weights
                 r_v=0.1, r_w=0.05,                 # effort weights
                 s_v=1.0, s_w=0.5,                  # comfort (rate) weights
                 qf_pos=50.0, qf_theta=2.0):        # terminal weights
        self.N, self.dt = N, dt

        opti = ca.Opti()
        X = opti.variable(3, N + 1)     # states over the horizon
        U = opti.variable(2, N)         # controls over the horizon
        x0 = opti.parameter(3)          # current state (from TF)
        u_prev = opti.parameter(2)      # last applied control (v, omega)
        ref = opti.parameter(3, N + 1)  # reference states, one per horizon step

        Q = ca.diag(ca.vertcat(q_pos, q_pos, q_theta))
        Qf = ca.diag(ca.vertcat(qf_pos, qf_pos, qf_theta))
        R = ca.diag(ca.vertcat(r_v, r_w))
        S = ca.diag(ca.vertcat(s_v, s_w))

        # --- cost -------------------------------------------------------
        J = 0
        for k in range(N + 1):          # tracking (terminal weight on the last)
            e = X[:, k] - ref[:, k]
            e = ca.vertcat(e[0], e[1], _wrap(e[2]))
            W = Qf if k == N else Q
            J += ca.mtimes([e.T, W, e])
        for k in range(N):              # effort + comfort (rate)
            J += ca.mtimes([U[:, k].T, R, U[:, k]])
            du = U[:, k] - (u_prev if k == 0 else U[:, k - 1])
            J += ca.mtimes([du.T, S, du])
        opti.minimize(J)

        # --- dynamics (multiple shooting) + initial condition ----------
        for k in range(N):
            xk, uk = X[:, k], U[:, k]
            x_next = xk + dt * ca.vertcat(uk[0] * ca.cos(xk[2]),
                                          uk[0] * ca.sin(xk[2]),
                                          uk[1])
            opti.subject_to(X[:, k + 1] == x_next)
        opti.subject_to(X[:, 0] == x0)

        # --- constraints: velocity + acceleration (rate) limits --------
        opti.subject_to(opti.bounded(-v_max, U[0, :], v_max))
        opti.subject_to(opti.bounded(-w_max, U[1, :], w_max))
        for k in range(N):
            du = U[:, k] - (u_prev if k == 0 else U[:, k - 1])
            opti.subject_to(opti.bounded(-dv_max, du[0], dv_max))
            opti.subject_to(opti.bounded(-dw_max, du[1], dw_max))

        opti.solver("ipopt", {"print_time": 0, "ipopt.print_level": 0,
                              "ipopt.sb": "yes", "ipopt.max_iter": 150})

        self.opti, self.X, self.U = opti, X, U
        self.p_x0, self.p_uprev, self.p_ref = x0, u_prev, ref
        self._Xg = self._Ug = None      # warm-start cache (last solution)

    def solve(self, state, u_prev, ref):
        """Solve one MPC step.
        Args:
            state: (3,) current [x, y, theta] (map frame).
            u_prev: (2,) last applied [v, omega].
            ref: (3, N+1) reference states over the horizon.
        Returns:
            (v, omega) — the first control to apply. (0,0) if the solve fails.
        """
        opti = self.opti
        opti.set_value(self.p_x0, state)
        opti.set_value(self.p_uprev, u_prev)
        opti.set_value(self.p_ref, ref)
        if self._Xg is not None:                     # warm start from last solution
            opti.set_initial(self.X, self._Xg)
            opti.set_initial(self.U, self._Ug)
        try:
            sol = opti.solve()
            self._Xg, self._Ug = sol.value(self.X), sol.value(self.U)
            u0 = self._Ug[:, 0]
        except RuntimeError:
            # infeasible / max-iter: fall back to the best iterate, else stop.
            try:
                u0 = opti.debug.value(self.U)[:, 0]
            except Exception:
                return (0.0, 0.0)
        return (float(u0[0]), float(u0[1]))
