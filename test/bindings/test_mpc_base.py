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
        mpc.setReferenceParameterizedInputTrajectory(u_traj_ctrl_pts=np.ones(nc))

        mpc.setInputLimits(u_min=-np.ones(m), u_max=np.ones(m))
        mpc.setStateLimits(x_min=-np.ones(n), x_max=np.ones(n))
        mpc.setSlewRate(u_slew=np.ones(m))
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
    assert hasattr(mpc, "getParameterizedInputTrajectory")
    assert hasattr(mpc, "getInputTrajectory")
    assert hasattr(mpc, "getPredictedStateTrajectory")

    assert function_fails(mpc.initializeSolver, solver_settings=None)
    assert function_fails(mpc.initializeSolver, solver_settings=ampc.OSQPSettings())

    assert function_fails(mpc.getNextInput, u0=None)
    assert function_fails(mpc.getParameterizedInputTrajectory, u_traj_ctrl_pts=None)
    assert function_fails(mpc.getInputTrajectory, u_traj=None)
    assert function_fails(mpc.getPredictedStateTrajectory, x_traj=None)


if __name__ == "__main__":
    test_mpc_base_interface()
    print("All tests passed!")
