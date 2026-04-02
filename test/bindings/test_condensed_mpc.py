import numpy as np

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
        mpc.setSlewRate(u_slew=np.ones(m))
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
        mpc.setReferenceParameterizedInputTrajectory(u_traj_ctrl_pts=np.ones(m * nc))

        mpc.initializeSolver()
        mpc.initializeSolver(solver_settings=ampc.OSQPSettings())

        mpc.solve(x0=np.zeros(n))

        u = mpc.getNextInput()
        address = id(u)
        out = mpc.getNextInput(u0=u)
        assert address == id(u)
        # can't test address of out, it is a different reference to same data
        verify_same_data(out, u)

        u_ctrl_pts = mpc.getParameterizedInputTrajectory()
        address = id(u_ctrl_pts)
        out = mpc.getParameterizedInputTrajectory(u_traj_ctrl_pts=u_ctrl_pts)
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
