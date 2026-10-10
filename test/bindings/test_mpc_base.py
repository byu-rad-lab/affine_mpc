import numpy as np
import pytest

import affine_mpc as ampc


def function_fails(func, *args, **kwargs):
    try:
        func(*args, **kwargs)
    except:
        return True
    return False


def test_mpc_base_interface():
    try:
        n, m, T, nc = 2, 1, 10, 5
        param = ampc.Parameterization.linearInterp(
            horizon_steps=T, num_control_points=nc
        )
        opts = ampc.Options()
        opts.use_input_cost = True
        opts.slew_control_points = True
        opts.slew_initial_input = True
        opts.saturate_states = True
        mpc = ampc.MPCBase(n, m, param, opts, m * nc, 0)

        mpc.setModelDiscrete(Ad=np.eye(n), Bd=np.ones(n), wd=np.zeros(n))
        mpc.setModelContinuous2Discrete(
            Ac=np.eye(n), Bc=np.ones(n), wc=np.zeros(n), dt=0.1
        )
        x_next = mpc.propagateModel(x=np.zeros(n), u=np.ones(m))
        mpc.propagateModel(x=np.zeros(n), u=np.ones(m), x_next=x_next)

        mpc.setWeights(Q_diag=np.ones(n), R_diag=np.ones(m))
        mpc.setWeights(Q_diag=np.ones(n), Qf_diag=np.ones(n), R_diag=np.ones(m))
        mpc.setStateWeights(Q_diag=np.ones(n))
        mpc.setStateWeights(Q_diag=np.ones(n), Qf_diag=np.ones(n))
        mpc.setInputWeights(R_diag=np.ones(m))

        mpc.setReferenceState(x_step=np.ones(n))
        mpc.setReferenceInput(u_step=np.ones(m))
        mpc.setReferenceStateTrajectory(x_traj=np.ones(T * n))
        mpc.setReferenceInputControlPoints(control_points=np.ones(nc))

        mpc.setInputLimits(u_min=-np.ones(m), u_max=np.ones(m))
        mpc.setStateLimits(x_min=-np.ones(n), x_max=np.ones(n))
        mpc.setSlewRate(control_point_slew=np.ones(m))
        mpc.setSlewRateInitial(u0_slew=np.ones(m))
        mpc.setPreviousInput(u_prev=np.zeros(m))

        _ = mpc.state_dim
        _ = mpc.input_dim
        _ = mpc.horizon_steps
        _ = mpc.num_control_points

    except:
        assert False

    assert mpc.solve(x0=np.zeros(n)) == ampc.SolveStatus.NotInitialized

    assert hasattr(mpc, "initializeSolver")
    assert hasattr(mpc, "getNextInput")
    assert hasattr(mpc, "getInputControlPoints")
    assert hasattr(mpc, "getInputTrajectory")
    assert hasattr(mpc, "getPredictedStateTrajectory")

    assert (
        repr(mpc)
        == "MPCBase(state_dim=2, input_dim=1, parameterization=Parameterization(horizon_steps=10, degree=1, num_control_points=5, knots=[0, 0, 2.25, 4.5, 6.75, 9, 9]), options=Options(use_input_cost=True, slew_initial_input=True, slew_control_points=True, saturate_states=True, saturate_input_trajectory=False), solver_initialized=False, Q=[1, 1], Qf=[1, 1], R=[1], u_min=[-1], u_max=[1], x_min=[-1, -1], x_max=[1, 1], u0_slew=[1], control_point_slew=[1])"
    )
    assert str(mpc).startswith("MPCBase:\n  state_dim = 2\n  input_dim = 1\n")

    assert function_fails(mpc.initializeSolver, solver_settings=None)
    assert function_fails(mpc.initializeSolver, solver_settings=ampc.OSQPSettings())

    assert function_fails(mpc.getNextInput, u0=None)
    assert function_fails(mpc.getInputControlPoints, control_points=None)
    assert function_fails(mpc.getInputTrajectory, u_traj=None)
    assert function_fails(mpc.getPredictedStateTrajectory, x_traj=None)


if __name__ == "__main__":
    test_mpc_base_interface()
    print("All tests passed!")


def _all_options_mpc(n=2, m=2, T=6, nc=3):
    param = ampc.Parameterization.linearInterp(horizon_steps=T, num_control_points=nc)
    opts = ampc.Options(
        use_input_cost=True,
        slew_initial_input=True,
        slew_control_points=True,
        saturate_states=True,
    )
    return ampc.CondensedMPC(n, m, param, opts), param, opts


def test_mpc_base_getters_return_set_values():
    n, m, T, nc = 2, 2, 6, 3
    mpc, _, opts = _all_options_mpc(n, m, T, nc)

    assert str(mpc.getOptions()) == str(opts)
    assert np.all(mpc.getInputMin() == -np.inf)
    assert np.all(mpc.getSlewRate() == np.inf)

    Q, Qf, R = np.array([1.0, 2.0]), np.array([3.0, 4.0]), np.array([0.5, 0.6])
    mpc.setWeights(Q_diag=Q, Qf_diag=Qf, R_diag=R)
    np.testing.assert_array_equal(mpc.getStateWeights(), Q)
    # the 3-argument setWeights used to drop Qf_diag
    np.testing.assert_array_equal(mpc.getTerminalStateWeights(), Qf)
    np.testing.assert_array_equal(mpc.getInputWeights(), R)

    mpc.setInputLimits(u_min=-np.ones(m), u_max=np.ones(m))
    mpc.setStateLimits(x_min=-5 * np.ones(n), x_max=5 * np.ones(n))
    mpc.setSlewRate(control_point_slew=np.array([0.1, 0.2]))
    mpc.setSlewRateInitial(u0_slew=np.array([0.3, 0.4]))
    mpc.setPreviousInput(u_prev=np.array([0.7, 0.8]))
    np.testing.assert_array_equal(mpc.getInputMin(), -np.ones(m))
    np.testing.assert_array_equal(mpc.getInputMax(), np.ones(m))
    np.testing.assert_array_equal(mpc.getStateMin(), -5 * np.ones(n))
    np.testing.assert_array_equal(mpc.getStateMax(), 5 * np.ones(n))
    np.testing.assert_array_equal(mpc.getSlewRate(), [0.1, 0.2])
    np.testing.assert_array_equal(mpc.getSlewRateInitial(), [0.3, 0.4])
    np.testing.assert_array_equal(mpc.getPreviousInput(), [0.7, 0.8])

    x_ref = np.linspace(0, 1, n * T)
    ctrls_ref = np.linspace(-1, 1, m * nc)
    mpc.setReferenceStateTrajectory(x_traj=x_ref)
    mpc.setReferenceInputControlPoints(control_points=ctrls_ref)
    np.testing.assert_array_equal(mpc.getReferenceStateTrajectory(), x_ref)
    np.testing.assert_array_equal(mpc.getReferenceInputControlPoints(), ctrls_ref)

    # getters return copies, so editing one does not change the MPC
    u_min = mpc.getInputMin()
    u_min[:] = 0.0
    np.testing.assert_array_equal(mpc.getInputMin(), -np.ones(m))


def test_mpc_base_optional_getters_raise_when_disabled():
    mpc = ampc.CondensedMPC(2, 1, ampc.Parameterization.linearInterp(5, 3))
    for getter in (
        mpc.getInputWeights,
        mpc.getReferenceInputControlPoints,
        mpc.getStateMin,
        mpc.getStateMax,
        mpc.getSlewRate,
        mpc.getSlewRateInitial,
        mpc.getPreviousInput,
    ):
        with pytest.raises(RuntimeError, match="not enabled"):
            getter()
