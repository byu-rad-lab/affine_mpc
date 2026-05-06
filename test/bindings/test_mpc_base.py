import numpy as np

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
