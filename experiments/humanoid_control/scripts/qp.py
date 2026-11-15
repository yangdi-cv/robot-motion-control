"""Two established QP backends for the exact same l <= A x <= u problem."""

from types import SimpleNamespace
import numpy as np
from scipy import sparse
import osqp
import clarabel


def solve_qp(H, g, A, l, u, backend):
    if backend == "osqp":
        solver = osqp.OSQP()
        solver.setup(
            P=sparse.csc_matrix(H),
            q=g,
            A=sparse.csc_matrix(A),
            l=l,
            u=u,
            verbose=False,
            eps_abs=1e-4,
            eps_rel=1e-4,
            max_iter=20000,
            polishing=True,
            adaptive_rho_interval=50,
        )
        solution = solver.solve(raise_error=False)
        if solution.info.status_val not in [1, 2]:
            raise RuntimeError(
                f"OSQP {solution.info.status}; primal={solution.info.prim_res:.3g}, dual={solution.info.dual_res:.3g}"
            )
        return solution
    if backend != "clarabel":
        raise ValueError(backend)
    equality = np.isfinite(l) & np.isfinite(u) & (l == u)
    below = np.isfinite(u) & ~equality
    above = np.isfinite(l) & ~equality
    matrix = sparse.csc_matrix(np.vstack([A[equality], A[below], -A[above]]))
    rhs = np.r_[u[equality], u[below], -l[above]]
    cones = [
        clarabel.ZeroConeT(int(equality.sum())),
        clarabel.NonnegativeConeT(int(below.sum() + above.sum())),
    ]
    settings = clarabel.DefaultSettings()
    settings.verbose = False
    settings.max_iter = 100
    settings.tol_feas = 1e-8
    settings.tol_gap_abs = 1e-8
    settings.tol_gap_rel = 1e-8
    solver = clarabel.DefaultSolver(
        sparse.triu(sparse.csc_matrix(H)).tocsc(), g, matrix, rhs, cones, settings
    )
    result = solver.solve()
    status = str(result.status)
    if status not in ["Solved", "AlmostSolved"]:
        raise RuntimeError(
            f"Clarabel {status}; primal={result.r_prim:.3g}, dual={result.r_dual:.3g}"
        )
    return SimpleNamespace(
        x=np.array(result.x),
        y=np.array(result.z),
        info=SimpleNamespace(iter=result.iterations, status=status),
    )
