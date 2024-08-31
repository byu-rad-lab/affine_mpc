import numpy as np

import affine_mpc_py as ampc


def test_mpc_base_interface():
    try:
        n,m,T,p = 2,1,10,5
        mpc = ampc.ImplicitMPC(num_states=n, num_inputs=m,
                               len_horizon=T, num_control_points=p,
                               use_input_cost=True, use_slew_rate=True,
                               saturate_states=True)

        mpc.setModelDiscrete(Ad=np.eye(n), Bd=np.ones(n), wd=np.zeros(n))
        mpc.setModelContinuous2Discrete(Ac=np.eye(n), Bc=np.ones(n),
                                        wc=np.zeros(n), dt=0.1)
        _ = mpc.propagateModel(x=np.zeros(n), u=np.ones(m))

        mpc.setInputLimits(u_min=-np.ones(m), u_max=np.ones(m))
        mpc.setStateLimits(x_min=-np.ones(n), x_max=np.ones(n))
        mpc.setSlewRate(u_slew=np.ones(m))

        mpc.setWeights(Q_diag=np.ones(n), R_diag=np.ones(m))
        mpc.setStateWeights(Q_diag=np.ones(n))
        mpc.setStateWeightsTerminal(Qf_diag=np.ones(n))
        mpc.setInputWeights(R_diag=np.ones(m))

        mpc.setReferenceState(x_step=np.ones(n))
        mpc.setReferenceInput(u_step=np.ones(m))
        mpc.setReferenceStateTrajectory(x_traj=np.ones(T*n))
        mpc.setReferenceParameterizedInputTrajectory(u_traj_ctrl_pts=np.ones(p))

        mpc.initializeSolver(solver_settings=None)

        mpc.solve(x0=np.zeros(n))

        u = mpc.getNextInput()
        address = id(u)
        _ = mpc.getNextInput(u0=u)
        assert address == id(u)

        u_ctrl_pts = mpc.getParameterizedInputTrajectory()
        address = id(u_ctrl_pts)
        _ = mpc.getParameterizedInputTrajectory(u_traj_ctrl_pts=u_ctrl_pts)
        assert address == id(u_ctrl_pts)

        u_traj = mpc.getInputTrajectory()
        address = id(u_traj)
        _ = mpc.getInputTrajectory(u_traj=u_traj)
        assert address == id(u_traj)

        x_traj = mpc.getPredictedStateTrajectory()
        address = id(x_traj)
        _ = mpc.getPredictedStateTrajectory(x_traj=x_traj)
        assert address == id(x_traj)

        _ = mpc.num_states
        _ = mpc.num_inputs
        _ = mpc.len_horizon
        _ = mpc.num_ctrl_pts

    except:
        assert False
