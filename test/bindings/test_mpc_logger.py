import numpy as np
import os

import affine_mpc_py as ampc


def test_mpc_logger_interface():
    try:
        folder = '/tmp/test_bindings/'
        paramfile = 'params.yaml'

        mpc = ampc.ImplicitMPC(num_states=2, num_inputs=1,
                               len_horizon=10, num_control_points=5)
        mpc.setModelContinuous2Discrete(Ac=np.eye(2), Bc=np.ones(2), wc=np.zeros(2), dt=0.1)
        mpc.setInputLimits(u_min=-np.ones(1), u_max=np.ones(1))
        mpc.initializeSolver()
        solved = mpc.solve(x0=np.zeros(2))

        logger = ampc.MPCLogger(mpc=mpc, save_location=folder)
        logger.logPreviousSolve(t0=0, ts=1, x0=np.zeros(2), solve_time=-1, write_every=1)
        logger.logPreviousSolve(t0=1, ts=1, x0=np.ones(2))
        logger.writeParamFile(filename='params.yaml')

        assert os.path.exists(folder + paramfile)
    except:
        assert False


if __name__ == "__main__":
    test_mpc_logger_interface()
    print("All tests passed!")
