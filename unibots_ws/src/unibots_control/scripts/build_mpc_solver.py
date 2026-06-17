#!/usr/bin/env python3
"""Offline iterative-MPC (iMPC) QP solver generator for the mecanum robot.

Run this ONCE on the development machine to emit a compiled OSQP C solver via
CVXPYgen. It is NOT executed at robot startup -- the Raspberry Pi 5 only ever
runs the generated, compiled code (``generated_solver/cpg_solver.py`` thin
wrapper + the OSQP C extension), avoiding all CVXPY/Python QP-construction
overhead at runtime.

================================================================================
MATHEMATICAL FORMULATION
================================================================================
State (arena/map frame):
    x = [px, py, theta]^T            (2D position [m] + heading [rad])

Control (body frame, holonomic mecanum):
    u = [vx, vy, omega]^T            (lateral vel, forward vel, yaw rate)

Continuous kinematics (holonomic / omnidirectional):
    pdot       = B(theta) @ u
    B(theta) = [[cos t, -sin t, 0],
                  [sin t,  cos t, 0],
                  [    0,      0, 1]]
B is the body-to-world rotation that maps body-frame velocities into world-frame
state derivatives. Forward Euler discretisation with step dt gives:
    x_{k+1} = x_k + dt * B(theta_k) @ u_k

WHY ITERATIVE LINEARISATION ("the i in iMPC"):
    The kinematics are NONLINEAR because B(theta) contains cos(theta)/sin(theta)
    multiplied by the decision variables u. A plain Linear MPC assumes a fixed
    linear model A x + B u and therefore CANNOT represent this trig coupling --
    a single global linearisation is only valid near one heading and degrades as
    the robot rotates. The iterative approach relinearises the model around the
    CURRENT operating point (the measured theta) at every control timestep, so
    each solve uses a model that is locally accurate. The structure of the QP
    (its variables, parameter shapes, cost, constraint sparsity) is constant;
    only the numeric Parameter values (x0, x_ref, B_k, obstacle half-planes)
    change each step. That fixed structure is exactly what lets us compile once
    and re-solve cheaply.

    NOTE: B(theta) is here exposed as N independent Parameters B_k so the node
    CAN, in future, supply a different linearisation per horizon step (true
    multiple-shooting relinearisation). The current node sets them all equal.

QP SOLVED EACH STEP:
    min  sum_{k=0}^{N-1} (x_k - x_ref_k)^T Q (x_k - x_ref_k)
                       + u_k^T R u_k
                       + du_k^T Rd du_k
       + (x_N - x_ref_N)^T Q (x_N - x_ref_N)          (terminal state cost)
    s.t. x_0 = x0
         x_{k+1} = x_k + dt * B_k @ u_k
         velocity box bounds on u
         arena wall box bounds on position
         obstacle half-planes: n_{k,i}^T p_k >= d_{k,i}
    where du_k = u_k - u_{k-1}  (du_0 = u_0 - 0, i.e. penalise jerk from rest).

================================================================================
WHY CVXPYgen
================================================================================
CVXPY builds and canonicalises the problem symbolically in Python, which is far
too slow to do at 20 Hz on a Pi 5. ``cvxpygen.cpg.generate_code`` performs that
canonicalisation ONCE here, offline, and emits standalone C that maps our named
Parameters straight into the OSQP data structures. At runtime the node just sets
parameter values and calls the compiled ``cpg_solve`` -- no CVXPY, no
re-canonicalisation, deterministic timing.

================================================================================
USAGE
================================================================================
    python3 scripts/build_mpc_solver.py

This need NOT be rerun unless the QP STRUCTURE changes (horizon, number of
obstacle slots, which quantities are Parameters, the cost/constraint topology).
Changing only numeric values at runtime does not require regeneration.
"""

import os
import sys

import cvxpygen.cpg as cpg

# The QP definition lives in the unibots_control package (mpc_problem.py) so the
# generator and the runtime node share EXACTLY one source of truth. Add the
# package's python dir to sys.path so this standalone script can import it
# without needing a sourced ROS environment.
_PKG_PY_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "unibots_control")
sys.path.insert(0, _PKG_PY_DIR)
from mpc_problem import build_problem  # noqa: E402  (after sys.path setup)

# NOTE: CVXPYgen treats `code_dir` as a *Python module path* -- it does
# `importlib.import_module(f'{code_dir}.cpg_solver')` during compilation -- so it
# must be a bare, importable directory name with NO trailing slash (otherwise the
# import becomes 'generated_solver/.cpg_solver' and fails with ModuleNotFoundError).
OUTPUT_CODE_DIR: str = "generated_solver"


def main() -> None:
    """Build the parametric QP and generate the compiled OSQP solver."""
    print("[build_mpc_solver] Building parametric iMPC QP ...")
    problem = build_problem()

    print(f"[build_mpc_solver] Generating OSQP C code into '{OUTPUT_CODE_DIR}' ...")
    cpg.generate_code(
        problem,
        code_dir=OUTPUT_CODE_DIR,
        solver="OSQP",
        wrapper=True,
    )

    print("=" * 72)
    print("iMPC solver generation COMPLETE.")
    print(f"  - Generated solver dir : {OUTPUT_CODE_DIR}")
    print("  - Python entry point   : generated_solver/cpg_solver.py")
    print("    Import it with:  from generated_solver.cpg_solver import cpg_solve")
    print("  - KEEP the 'generated_solver/' directory next to the controller node")
    print("    (or on PYTHONPATH) so mpc_controller_node.py can import it.")
    print("  - You do NOT need to rerun this unless the QP STRUCTURE changes")
    print("    (horizon, obstacle slot count, parameter layout, cost/constraint")
    print("    topology). Numeric value changes at runtime do not require it.")
    print("=" * 72)


if __name__ == "__main__":
    main()
