#!/usr/bin/env python3
"""Shared iMPC QP definition for the Unibots mecanum robot.

This module is the SINGLE SOURCE OF TRUTH for the iterative-MPC quadratic
program. It is imported by BOTH:

  * scripts/build_mpc_solver.py  -- which passes the CVXPY problem to CVXPYgen
    to emit the compiled OSQP C solver (offline, once), and
  * unibots_control/mpc_controller_node.py -- which, at runtime, rebuilds an
    identical CVXPY ``Problem`` and binds the compiled ``cpg_solve`` to it.

Why the node must rebuild the problem
-------------------------------------
CVXPYgen's generated ``cpg_solver.py`` exposes only the ``cpg_solve(problem)``
*function*; it does NOT contain a ``Problem`` object. ``cpg_solve`` uses the
CVXPY problem purely as a typed container for the named ``param_dict`` /
``var_dict`` -- it pushes the parameter values into the precompiled C data,
runs the compiled OSQP solve (no re-canonicalisation, the whole point of
CVXPYgen), and writes the optimal variables back onto the problem. So the node
needs a CVXPY problem with the SAME parameter/variable names and shapes used at
generation time -- which is exactly what calling :func:`build_problem` here
guarantees. (CVXPY is imported at runtime only to hold this container; the
numerical work is done by the compiled solver.)

Pure CVXPY/NumPy -- no ROS dependencies -- so the offline generator can import
it without a sourced ROS environment.
"""

from typing import Tuple

import cvxpy as cp
import numpy as np

# ----------------------------------------------------------------------------
# Named constants (NO magic numbers). MUST match between generation and runtime.
# ----------------------------------------------------------------------------
N_STATES: int = 3                    # [px, py, theta]
N_CONTROLS: int = 3                  # [vx, vy, omega]
HORIZON_N: int = 10                  # prediction horizon (steps)
DT: float = 0.05                     # discretisation timestep [s] -> 20 Hz

# Maximum number of dynamic-obstacle half-planes per horizon step. The node
# always populates exactly this many; unused slots are made trivially satisfied.
MAX_OBSTACLES: int = 6

# Velocity bounds (body frame). Conservative for foam-tile slip on the arena.
MAX_VX: float = 2.0                # body lateral velocity [m/s]
MAX_VY: float = 2.0                 # body forward velocity [m/s]
MAX_OMEGA: float = 3.0               # yaw rate [rad/s]

# Cost weights.
Q_DIAG = (10.0, 10.0, 0.5)           # state tracking (px, py, theta)
R_DIAG = (0.5, 0.5, 0.3)             # control effort (vx, vy, omega)
RD_DIAG = (1.5, 1.5, 0.8)            # control rate / smoothness (dvx, dvy, domega)

# Soft-constraint slack penalties. Large relative to the tracking cost so the
# obstacle constraints behave as effectively hard whenever a
# feasible interior solution exists, while never making the QP infeasible.
OBS_SLACK_PENALTY: float = 1.0e4     # per-metre penalty on obstacle violation

# Inactive-obstacle sentinel half-plane: n=[1,0], d=-999 so the constraint
# n^T p >= d is satisfied everywhere in the arena.
INACTIVE_NORMAL: Tuple[float, float] = (1.0, 0.0)
INACTIVE_D: float = -999.0


def build_problem() -> cp.Problem:
    """Construct the parametric CVXPY QP for the iMPC controller.

    The structure (variables, parameter names/shapes, cost, constraints) is
    fixed; only the numeric Parameter values change at runtime. Returns a
    :class:`cvxpy.Problem` whose ``param_dict`` / ``var_dict`` expose:

    Parameters (set each control tick):
        x0      (3,)        current state
        x_ref   (3, N+1)    reference trajectory
        B_k     (3, 3)      linearised body-to-world matrix at step k (k=0..N-1)
        n_k_i   (2,)        obstacle i half-plane normal at step k
        d_k_i   scalar      obstacle i half-plane offset at step k

    Variables:
        u       (3, N)      control sequence (u[:,0] is applied)
        x       (3, N+1)    predicted state trajectory
        s_wall  (4, N+1)    arena-wall slack (soft constraint)
        s_obs   (M, N+1)    obstacle slack (soft constraint)
    """
    # --- Decision variables ---------------------------------------------------
    u = cp.Variable((N_CONTROLS, HORIZON_N), name="u")        # controls
    x = cp.Variable((N_STATES, HORIZON_N + 1), name="x")      # states

    # --- Runtime parameters ---------------------------------------------------
    x0 = cp.Parameter(N_STATES, name="x0")                            # current state
    x_ref = cp.Parameter((N_STATES, HORIZON_N + 1), name="x_ref")     # reference traj

    B_mats = [cp.Parameter((N_STATES, N_STATES), name=f"B_{k}")
              for k in range(HORIZON_N)]

    # Obstacle half-planes: n_{k,i}^T p_k >= d_{k,i}.
    obs_normals = [[cp.Parameter(2, name=f"n_{k}_{i}")
                    for i in range(MAX_OBSTACLES)]
                   for k in range(HORIZON_N + 1)]
    # Scalar (no shape arg) to avoid 1D broadcasting issues.
    obs_offsets = [[cp.Parameter(name=f"d_{k}_{i}")
                    for i in range(MAX_OBSTACLES)]
                   for k in range(HORIZON_N + 1)]

    # --- Fixed cost matrices --------------------------------------------------
    Q = np.diag(Q_DIAG)
    R = np.diag(R_DIAG)
    Rd = np.diag(RD_DIAG)

    # --- Soft-constraint slack variables (>= 0) -------------------------------
    # Making the arena walls and obstacle half-planes soft (penalised slack)
    # guarantees the QP is ALWAYS feasible -- critical because the robot can
    # legitimately start or sit slightly outside the nominal interior (e.g. the
    # (0.1, 0.1) corner spawn, inside the 0.15 m wall buffer), or be given a goal
    # at a wall. With HARD bounds those cases make the problem infeasible and the
    # controller outputs nothing. The large penalty keeps slack ~0 whenever an
    # interior solution exists, so normal behaviour matches hard constraints.
    s_obs = cp.Variable((MAX_OBSTACLES, HORIZON_N + 1), name="s_obs", nonneg=True)

    # --- Constraints ----------------------------------------------------------
    constraints = [x[:, 0] == x0]

    for k in range(HORIZON_N):
        constraints += [x[:, k + 1] == x[:, k] + DT * (B_mats[k] @ u[:, k])]
        constraints += [
            cp.abs(u[0, k]) <= MAX_VX,
            cp.abs(u[1, k]) <= MAX_VY,
            cp.abs(u[2, k]) <= MAX_OMEGA,
        ]

    for k in range(HORIZON_N + 1):
        # Soft obstacle half-planes.
        for i in range(MAX_OBSTACLES):
            constraints += [
                obs_normals[k][i] @ x[0:2, k] >= obs_offsets[k][i] - s_obs[i, k]
            ]

    # --- Cost -----------------------------------------------------------------
    cost = 0
    for k in range(HORIZON_N):
        # quad_form manually expanded so the parameter-dependent term stays
        # affine in the variable: (x-x_ref)^T Q (x-x_ref) = x^T Q x - 2 x_ref^T Q x
        # (the constant x_ref^T Q x_ref does not affect the argmin and is dropped).
        cost += cp.quad_form(x[:, k], Q) - 2 * x_ref[:, k] @ Q @ x[:, k]
        cost += cp.quad_form(u[:, k], R)
        du = u[:, k] if k == 0 else u[:, k] - u[:, k - 1]
        cost += cp.quad_form(du, Rd)

    # Terminal state cost (also manually expanded).
    cost += cp.quad_form(x[:, HORIZON_N], Q) - 2 * x_ref[:, HORIZON_N] @ Q @ x[:, HORIZON_N]

    # Soft-constraint slack penalties (L1).
    cost += OBS_SLACK_PENALTY * cp.sum(s_obs)

    return cp.Problem(cp.Minimize(cost), constraints)
