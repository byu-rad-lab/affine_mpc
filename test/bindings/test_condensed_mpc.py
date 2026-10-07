import numpy as np
import pytest

import affine_mpc as ampc


def verify_same_data(a, b):
    # modify in place
    a += 1
    assert np.all(a == b)


def test_implicit_mpc_interface():
    try:
        n, m, T, nc = 2, 1, 10, 5

        mpc = ampc.CondensedMPC(state_dim=n, input_dim=m, horizon_steps=T)
        mpc = ampc.CondensedMPC(
            state_dim=n, input_dim=m, horizon_steps=T, opts=ampc.Options()
        )

        mpc = ampc.CondensedMPC(
            state_dim=n,
            input_dim=m,
            param=ampc.Parameterization.linearInterp(
                horizon_steps=T, num_control_points=nc
            ),
            opts=ampc.Options(
                use_input_cost=True,
                slew_initial_input=True,
                slew_control_points=True,
                saturate_states=True,
            ),
        )

        mpc.setModelDiscrete(Ad=np.eye(n), Bd=np.ones(n), wd=np.zeros(n))
        mpc.setModelContinuous2Discrete(
            Ac=np.eye(n), Bc=np.ones(n), wc=np.zeros(n), dt=0.1
        )
        _ = mpc.propagateModel(x=np.zeros(n), u=np.ones(m))

        mpc.setInputLimits(u_min=-np.ones(m), u_max=np.ones(m))
        mpc.setStateLimits(x_min=-np.ones(n), x_max=np.ones(n))
        mpc.setSlewRate(control_point_slew=np.ones(m))
        mpc.setSlewRateInitial(u0_slew=np.ones(m))
        mpc.setPreviousInput(u_prev=np.zeros(m))

        mpc.setWeights(Q_diag=np.ones(n), R_diag=np.ones(m))
        mpc.setWeights(Q_diag=np.ones(n), Qf_diag=np.ones(n), R_diag=np.ones(m))
        mpc.setStateWeights(Q_diag=np.ones(n))
        mpc.setStateWeights(Q_diag=np.ones(n), Qf_diag=np.ones(n))
        mpc.setInputWeights(R_diag=np.ones(m))

        mpc.setReferenceState(x_step=np.ones(n))
        mpc.setReferenceInput(u_step=np.ones(m))
        mpc.setReferenceStateTrajectory(x_traj=np.ones(T * n))
        mpc.setReferenceInputControlPoints(control_points=np.ones(m * nc))

        mpc.initializeSolver()
        mpc.initializeSolver(solver_settings=ampc.OSQPSettings())

        mpc.solve(x0=np.zeros(n))

        u = mpc.getNextInput()
        address = id(u)
        out = mpc.getNextInput(u0=u)
        assert address == id(u)
        # can't test address of out, it is a different reference to same data
        verify_same_data(out, u)

        u_ctrl_pts = mpc.getInputControlPoints()
        address = id(u_ctrl_pts)
        out = mpc.getInputControlPoints(control_points=u_ctrl_pts)
        assert address == id(u_ctrl_pts)
        verify_same_data(out, u_ctrl_pts)

        u_traj = mpc.getInputTrajectory()
        address = id(u_traj)
        out = mpc.getInputTrajectory(u_traj=u_traj)
        assert address == id(u_traj)
        verify_same_data(out, u_traj)

        x_traj = mpc.getPredictedStateTrajectory()
        address = id(x_traj)
        out = mpc.getPredictedStateTrajectory(x_traj=x_traj)
        assert address == id(x_traj)
        verify_same_data(out, x_traj)

        _ = mpc.state_dim
        _ = mpc.input_dim
        _ = mpc.horizon_steps
        _ = mpc.num_control_points

    except:
        assert False


if __name__ == "__main__":
    test_implicit_mpc_interface()
    print("All tests passed!")


def test_qp_cost_matrix_is_copy():
    nc = 5
    mpc = ampc.CondensedMPC(2, 1, ampc.Parameterization.linearInterp(10, nc))
    mpc.setModelDiscrete(
        np.array([[1.0, 0.1], [-0.06, 0.99]]), np.array([0.0, 0.02]), np.zeros(2)
    )
    mpc.setInputLimits(np.array([-1.0]), np.array([1.0]))
    mpc.setReferenceState(np.array([1.0, 0.0]))
    assert mpc.initializeSolver()
    assert mpc.solve(np.zeros(2)) == ampc.SolveStatus.Success

    P = mpc.getQPCostMatrix()
    assert P.shape == (nc, nc)
    assert np.allclose(P, P.T)

    P[0, 0] = 1e6  # modifying the copy must not affect the MPC
    assert mpc.getQPCostMatrix()[0, 0] != 1e6


@pytest.mark.parametrize("mpc_type", [ampc.CondensedMPC, ampc.SparseMPC])
def test_solution_getters_before_initialize_raise(mpc_type):
    mpc = mpc_type(2, 1, 10)
    assert not mpc.isSolverInitialized()

    getters = [
        mpc.getNextInput,
        mpc.getInputControlPoints,
        mpc.getInputTrajectory,
        mpc.getPredictedStateTrajectory,
    ]
    for getter in getters:
        with pytest.raises(RuntimeError, match="must be initialized"):
            getter()

    with pytest.raises(RuntimeError, match="must be initialized"):
        mpc.getNextInput(np.zeros(1))


@pytest.mark.parametrize("mpc_type", [ampc.CondensedMPC, ampc.SparseMPC])
def test_wrong_size_arguments_raise(mpc_type):
    n, m, T = 2, 1, 10
    mpc = mpc_type(n, m, T)

    # setters are checked in C++ and raise ValueError (std::invalid_argument)
    with pytest.raises(ValueError, match="x_step must have size 2, got 5"):
        mpc.setReferenceState(np.ones(5))
    with pytest.raises(ValueError, match="Q_diag must have size 2, got 7"):
        mpc.setStateWeights(np.ones(7))
    with pytest.raises(ValueError, match="Ad must be 2x2, got 3x3"):
        mpc.setModelDiscrete(np.eye(3), np.ones(n), np.zeros(n))

    mpc.setModelDiscrete(
        np.array([[1.0, 0.1], [-0.06, 0.99]]), np.array([0.0, 0.02]), np.zeros(n)
    )
    with pytest.raises(ValueError, match="x must have size 2"):
        mpc.propagateModel(np.zeros(3), np.zeros(m))

    mpc.setInputLimits(np.array([-1.0]), np.array([1.0]))
    mpc.setStateWeights(np.ones(n))
    mpc.setReferenceState(np.array([1.0, 0.0]))
    assert mpc.initializeSolver()

    # solve() and the output-buffer getters are checked in the bindings
    with pytest.raises(ValueError, match="x0 must have size 2, got 3"):
        mpc.solve(np.zeros(3))
    assert mpc.solve(np.zeros(n)) == ampc.SolveStatus.Success

    with pytest.raises(ValueError, match="u0 must have size 1, got 4"):
        mpc.getNextInput(np.zeros(4))
    with pytest.raises(ValueError, match="control_points must have size"):
        mpc.getInputControlPoints(np.zeros(1))
    with pytest.raises(ValueError, match="u_traj must have size"):
        mpc.getInputTrajectory(np.zeros(1))
    with pytest.raises(ValueError, match="x_traj must have size"):
        mpc.getPredictedStateTrajectory(np.zeros(1))


@pytest.mark.parametrize("mpc_type", [ampc.CondensedMPC, ampc.SparseMPC])
def test_reset_warm_start_repeats_solve_from_zero(mpc_type):
    mpc = mpc_type(2, 1, ampc.Parameterization.linearInterp(10, 5))
    mpc.resetWarmStart()  # no-op before initialization
    mpc.setModelDiscrete(
        np.array([[1.0, 0.1], [-0.06, 0.99]]), np.array([0.0, 0.02]), np.zeros(2)
    )
    mpc.setInputLimits(np.array([-1.0]), np.array([1.0]))
    mpc.setStateWeights(np.ones(2))
    mpc.setReferenceState(np.array([1.0, 0.0]))

    settings = ampc.OSQPSettings()
    settings.adaptive_rho = False  # deterministic iteration counts
    settings.check_termination = 1
    assert mpc.initializeSolver(settings)

    x0 = np.array([0.5, -0.2])
    assert mpc.solve(x0) == ampc.SolveStatus.Success
    iters_from_zero = mpc.getSolveInfo().iterations
    u_first = mpc.getNextInput()

    assert mpc.solve(x0) == ampc.SolveStatus.Success
    assert mpc.getSolveInfo().iterations < iters_from_zero
    u_warm = mpc.getNextInput()

    mpc.resetWarmStart()
    assert np.array_equal(mpc.getNextInput(), u_warm)

    assert mpc.solve(x0) == ampc.SolveStatus.Success
    assert mpc.getSolveInfo().iterations == iters_from_zero
    assert np.array_equal(mpc.getNextInput(), u_first)
