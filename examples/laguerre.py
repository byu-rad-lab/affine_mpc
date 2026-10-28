"""Laguerre input parameterization with Parameterization.fromBasis().

Discrete Laguerre functions decay like pole**k, so an input u = Psi @ c decays
to zero over the horizon and can not hold the steady input that keeps the mass
at a nonzero reference. The deviation form u = u_ss + Psi @ c fixes this: u_ss
is the steady-state input that holds the reference, and it enters the model
through the offset w, so the MPC only plans the deviation from it.

This runs the mass-spring-damper of sim.py in closed loop with both forms and
prints the tracking cost of each. It only needs numpy.
"""

import numpy as np

import affine_mpc as ampc


def laguerre_basis(horizon_steps: int, num_functions: int, pole: float) -> np.ndarray:
    """Discrete Laguerre functions (Wang, 2004) sampled at k = 0, ..., horizon_steps-1.

    Returns Psi of shape (horizon_steps, num_functions): row k is L(k), with
    L(0) = sqrt(1 - a^2) [1, -a, a^2, ...] and L(k+1) = Omega L(k).
    """
    a, beta = pole, 1.0 - pole**2
    omega = np.zeros((num_functions, num_functions))
    for i in range(num_functions):
        omega[i, i] = a
        for j in range(i):
            omega[i, j] = beta * (-a) ** (i - j - 1)
    L = np.sqrt(beta) * (-a) ** np.arange(num_functions)
    basis = np.zeros((horizon_steps, num_functions))
    for k in range(horizon_steps):
        basis[k] = L
        L = omega @ L
    return basis


def prepare_basis(basis: np.ndarray) -> np.ndarray:
    """Orthonormalize the columns.

    Orthonormal columns (same span, so the same input trajectories) keep the
    QP well conditioned even for slow poles and many functions. The rows that
    have decayed to round-off need no handling: the MPC omits their input
    bound rows.
    """
    q, _ = np.linalg.qr(basis)
    return q


def run(deviation: bool) -> tuple[float, float]:
    """Closed-loop run; returns the final position and the tracking cost."""
    T, N, pole, ts = 40, 4, 0.8, 0.1
    param = ampc.Parameterization.fromBasis(T, prepare_basis(laguerre_basis(T, N, pole)))
    mpc = ampc.MPC(state_dim=2, input_dim=1, param=param, opts=ampc.Options(input_cost=ampc.InputCostMode.ControlPoints))

    A = np.array([[0.0, 1.0], [-0.6, -0.1]])
    B = np.array([0.0, 0.2])
    w = np.zeros(2)
    x_ref = np.array([1.0, 0.0])
    u_min, u_max = np.array([0.0]), np.array([4.0])

    # steady-state input that holds x_ref: 0 = A x_ref + B u_ss + w
    u_ss = np.linalg.lstsq(B[:, None], -(A @ x_ref + w), rcond=None)[0]
    if not deviation:
        u_ss = np.zeros(1)  # direct form: u = Psi @ c

    # the MPC plans u_tilde = u - u_ss: the offset moves into the model, and
    # the input limits shift by u_ss
    mpc.setModelContinuous2Discrete(A, B, w + B * u_ss[0], ts)
    mpc.setInputLimits(u_min - u_ss, u_max - u_ss)
    Q_diag = np.array([1.0, 0.1])
    mpc.setWeights(Q_diag, np.array([1e-3]))  # weights the Laguerre coefficients
    mpc.setReferenceState(x_ref)
    if not mpc.initializeSolver():
        raise RuntimeError("Failed to initialize solver")

    xk = np.zeros(2)
    cost = 0.0
    for _ in range(100):
        status = mpc.solve(xk)
        if status != ampc.SolveStatus.Success:
            print("Solver status:", status)
        u_tilde = mpc.getNextInput()
        uk = u_ss + u_tilde  # input applied to the plant, within [u_min, u_max]
        # The MPC's model includes the offset B u_ss, so propagating it with
        # u_tilde is the same as propagating the plant with uk.
        xk = mpc.propagateModel(xk, u_tilde)
        cost += float((xk - x_ref) @ (Q_diag * (xk - x_ref)))
    print(f"{'deviation' if deviation else 'direct':9} form: final position {xk[0]:.4f}, cost {cost:.3f}")
    return xk[0], cost


def main():
    run(deviation=False)
    run(deviation=True)


if __name__ == "__main__":
    main()
