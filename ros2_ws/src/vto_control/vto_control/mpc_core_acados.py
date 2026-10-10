"""Nonlinear MPC core for a differential-drive (unicycle) robot, solved via
acados (SQP_RTI) instead of CasADi+IPOPT. NO ROS deps.

Same interface as mpc_core.MPCCore: builds the OCP ONCE (acados code-generates and
compiles a solver for it), re-solves each tick with new parameter values. See
docs/27 (why acados) and docs/28 (this translation).

Formulation differences from mpc_core.py, both needed because acados' per-stage
NONLINEAR_LS cost only sees (x_k, u_k, p_k) -- never another stage's variables:
  - State is AUGMENTED with (v_prev, w_prev): "the control applied at the previous
    stage", propagated by the discrete dynamics as x_prev_{k+1} = u_k. This turns
    the control-RATE cost/constraint (u_k - u_{k-1} in mpc_core.py) into a same-
    stage residual (u_k - x_prev_k).
  - The heading reference is carried as a per-stage PARAMETER p = (ref_x, ref_y,
    ref_theta), and the wrapped heading error atan2(sin(theta-p2), cos(theta-p2))
    is computed INSIDE the cost expression (yref stays zero), because acados
    costs are (y_expr(x,u,p) - yref)^2 with a fixed numeric yref, not a wrapped
    numeric difference like mpc_core.py computes directly.
  - Dynamics use disc_dyn_expr (an explicit discrete map), matching mpc_core.py's
    exact forward-Euler stage update -- not an ODE + integrator, which would add
    an integration-scheme choice that doesn't exist in the IPOPT baseline.
"""
import shutil
import tempfile

import numpy as np
from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
from casadi import SX, atan2, cos, sin, vertcat


class MPCCoreAcados:
    def __init__(self, N=20, dt=0.1,
                 v_max=0.5, v_min=0.0, w_max=1.5, dv_max=0.15, dw_max=0.30,
                 q_pos=10.0, q_theta=0.5,           # tracking weights
                 r_v=0.1, r_w=0.05,                 # effort weights
                 s_v=1.0, s_w=0.5,                  # comfort (rate) weights
                 qf_pos=50.0, qf_theta=2.0,         # terminal weights
                 max_curvature=None):
        """max_curvature=None (default): unchanged unicycle behavior -- (v, w)
        are independently bounded by [v_min,v_max] x [-w_max,w_max], correct
        for a diff-drive robot (docs/13/27/28). max_curvature=tan(delta_max)/
        wheelbase (docs/35's formula) switches to the Ackermann/bicycle case:
        w is NOT independently steerable -- it's tied to v via the steering
        angle, so the real constraint is |w| <= max_curvature * v, a
        SPEED-DEPENDENT bound tighter than any fixed w_max at low speed and
        looser at high speed. The dynamics model itself does NOT change: a
        unicycle (x, y, theta) driven by (v, w) is the correct abstraction
        for ANY steered vehicle once you're commanding (v, w) -- Ackermann
        vs. diff-drive differs only in which (v, w) pairs are physically
        realizable, exactly the same idea pure_pursuit_ackermann_node.py
        already applies via a post-hoc clamp (docs/35); here it's an NLP
        constraint instead, so the optimizer never PLANS a violation instead
        of being corrected after the fact.
        """
        self.N, self.dt = N, dt
        self._tmpdir = tempfile.mkdtemp(prefix="acados_mpc_")  # container-local,
        # never under the bind-mounted /workspace -- avoids root-owned generated
        # code leaking into the repo (same class of issue as the colcon gotcha).

        # --- augmented state: (x, y, theta, v_prev, w_prev) -----------------
        x, y, theta, v_prev, w_prev = (SX.sym(n) for n in
                                        ("x", "y", "theta", "v_prev", "w_prev"))
        xa = vertcat(x, y, theta, v_prev, w_prev)
        v, w = SX.sym("v"), SX.sym("w")
        u = vertcat(v, w)
        p = SX.sym("p", 3)                          # per-stage ref: (rx, ry, rtheta)

        model = AcadosModel()
        model.disc_dyn_expr = vertcat(
            x + dt * v * cos(theta),
            y + dt * v * sin(theta),
            theta + dt * w,
            v,                                       # v_prev_{k+1} = v_k
            w,                                        # w_prev_{k+1} = w_k
        )
        model.x, model.u, model.p, model.name = xa, u, p, "vto_mpc_unicycle"

        nx, nu = 5, 2
        ny, ny_e = 7, 3                              # stage / terminal residual dims

        heading_err = atan2(sin(theta - p[2]), cos(theta - p[2]))  # wrap (docs/28 §1c)
        dv, dw = v - v_prev, w - w_prev              # same-stage rate residual (§1b)

        ocp = AcadosOcp()
        ocp.model = model
        ocp.dims.N = N
        ocp.solver_options.N_horizon = N
        ocp.solver_options.tf = N * dt
        ocp.parameter_values = np.zeros(3)

        ocp.cost.cost_type, ocp.cost.cost_type_e = "NONLINEAR_LS", "NONLINEAR_LS"
        ocp.model.cost_y_expr = vertcat(x - p[0], y - p[1], heading_err, v, w, dv, dw)
        ocp.model.cost_y_expr_e = vertcat(x - p[0], y - p[1], heading_err)
        ocp.cost.yref, ocp.cost.yref_e = np.zeros(ny), np.zeros(ny_e)
        ocp.cost.W = np.diag([q_pos, q_pos, q_theta, r_v, r_w, s_v, s_w])
        ocp.cost.W_e = np.diag([qf_pos, qf_pos, qf_theta])

        # hard bounds: control limits + rate limits (same-stage nonlinear constraint)
        ocp.constraints.lbu = np.array([v_min, -w_max])
        ocp.constraints.ubu = np.array([v_max, w_max])
        ocp.constraints.idxbu = np.array([0, 1])
        h_terms = [dv, dw]
        lh_terms = [-dv_max, -dw_max]
        uh_terms = [dv_max, dw_max]
        if max_curvature is not None:
            # |w| <= max_curvature * v, i.e. w - max_curvature*v <= 0 AND
            # -w - max_curvature*v <= 0 -- valid since v_min=0 keeps v >= 0
            # (forward-only, already this project's convention, docs/14)
            # so max_curvature*v is always a well-defined nonneg bound.
            # -10.0, not -np.inf: acados' code generator serializes the OCP
            # to JSON, which has no Infinity token (-np.inf silently produced
            # invalid JSON the renderer choked on); -10 is already far
            # outside [v,w]'s reachable range here, so it never binds.
            h_terms += [w - max_curvature * v, -w - max_curvature * v]
            lh_terms += [-10.0, -10.0]
            uh_terms += [0.0, 0.0]
        model.con_h_expr = vertcat(*h_terms)
        ocp.constraints.lh = np.array(lh_terms)
        ocp.constraints.uh = np.array(uh_terms)
        # Stage 0 needs its OWN copy (con_h_expr_0/lh_0/uh_0) -- acados
        # defaults it to UNCONSTRAINED if unset, so without this, the very
        # control solve() actually returns (stage 0's u) silently ignored
        # every h-constraint, box bounds aside. Found by direct measurement:
        # the returned (v,w) tracked the outer box bound exactly, and the
        # pre-existing dv/dw rate constraint had a small but real tick-0
        # violation too -- both point at the same missing stage-0 copy, not
        # a bug specific to the new curvature terms.
        model.con_h_expr_0 = model.con_h_expr
        ocp.constraints.lh_0 = ocp.constraints.lh
        ocp.constraints.uh_0 = ocp.constraints.uh
        ocp.constraints.x0 = np.zeros(nx)            # overwritten every solve() call

        ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"  # exploits shooting
        ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
        ocp.solver_options.integrator_type = "DISCRETE"
        ocp.solver_options.nlp_solver_type = "SQP_RTI"  # docs/27: bounded per-tick cost
        ocp.solver_options.print_level = 0           # explicit, not just relying on the default
        # qp_solver_iter_max default (50) already caps the inner QP's worst case --
        # checked, not left unexamined; no evidence (docs/28 step 4) that it needs
        # lowering, and our margin under the control period is already large.
        #
        # SQP_RTI does exactly ONE QP solve per solve() call, which makes that QP
        # "the first QP" of every NLP solve by acados' own accounting -- so the
        # default (nlp_solver_warm_start_first_qp=False) means the inner QP was
        # cold-starting every tick even though the outer trajectory guess (X, U)
        # persists across calls. This closes that gap: seed the QP from the
        # persisted NLP iterate (docs/29).
        ocp.solver_options.qp_solver_warm_start = 2                    # HPIPM "hot start"
        ocp.solver_options.nlp_solver_warm_start_first_qp = True
        ocp.solver_options.nlp_solver_warm_start_first_qp_from_nlp = True
        ocp.solver_options.qp_solver_cond_N = N       # required for the option above
        ocp.code_export_directory = f"{self._tmpdir}/c_generated_code"

        # (json_file kwarg is ignored by this acados_template version -- it always
        # writes into code_export_directory, which is already our tmpdir above)
        self.solver = AcadosOcpSolver(ocp)
        self.U_seq = np.zeros((2, N))                # last predicted sequence (docs/28)
        self.last_status = None

    def __del__(self):
        shutil.rmtree(self._tmpdir, ignore_errors=True)

    def solve(self, state, u_prev, ref):
        """Solve one MPC step.
        Args:
            state: (3,) current [x, y, theta] (map frame).
            u_prev: (2,) last applied [v, omega].
            ref: (3, N+1) reference states over the horizon.
        Returns:
            (v, omega) -- the first control to apply. (0,0) if the solve fails.
        """
        x0 = np.concatenate([np.asarray(state, dtype=float), np.asarray(u_prev, dtype=float)])
        self.solver.set(0, "lbx", x0)
        self.solver.set(0, "ubx", x0)
        for k in range(self.N + 1):
            self.solver.set(k, "p", np.asarray(ref[:, k], dtype=float))

        self.last_status = self.solver.solve()  # 0 = success; surfaced for the caller to log
        self.last_qp_iter = self.solver.get_stats("qp_iter")  # docs/29: warm-start diagnostics
        try:
            for k in range(self.N):
                self.U_seq[:, k] = self.solver.get(k, "u")
            u0 = self.U_seq[:, 0]
        except Exception:
            return (0.0, 0.0)
        return (float(u0[0]), float(u0[1]))
