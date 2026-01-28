import numpy as np
import affine_mpc_py as ampc


def main():
    msd_mpc = ampc.ImplicitMPC(state_dim=2, input_dim=1,
                               horizon_steps=10, num_control_points=3,
                               use_input_cost=True, use_slew_rate=True)

    logger = ampc.MPCLogger(msd_mpc, "/tmp/ampc_example")

    A = np.array([[0,1], [-0.6,-0.1]])
    B = np.array([0,0.2])
    w = np.zeros(2)
    ts = 0.1
    msd_mpc.setModelContinuous2Discrete(A, B, w, ts)

    u_min = np.zeros(1)
    u_max = np.ones(1) * 3
    msd_mpc.setInputLimits(u_min, u_max)

    slew = np.ones(1)
    msd_mpc.setSlewRate(slew)

    Q_diag = np.array([1,0.11])
    R_diag = np.array([.0001])
    msd_mpc.setWeights(Q_diag, R_diag)

    x_goal = np.array([1.0,0])
    u_goal = np.array([0.0])
    msd_mpc.setReferenceState(x_goal)
    msd_mpc.setReferenceInput(u_goal)

    msd_mpc.initializeSolver()

    xk = np.zeros(2)
    t = 0.0
    tf = 5.0
    while t < tf:
        solved = msd_mpc.solve(xk)
        if not solved:
            print('Did not solve :(')
        uk = msd_mpc.getNextInput()
        logger.logPreviousSolve(t, ts, xk)
        xk = msd_mpc.propagateModel(xk, uk)
        t += ts

    logger.writeParamFile()


if __name__ == "__main__":
    main()
