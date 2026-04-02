import numpy as np
import affine_mpc as ampc


def test_parameterization_interface():
    try:
        T, nc = 5, 3

        param = ampc.Parameterization.moveBlocking(
            horizon_steps=T, num_control_points=nc
        )
        param = ampc.Parameterization.moveBlocking(
            horizon_steps=T, change_points=np.array([0, 2, 4])
        )

        m = 2
        ctrls = np.array([[-1.0, 1], [0, 0], [1, -1]])
        u_traj = param.evaluate(input_dim=m, control_points=ctrls.ravel())

        param = ampc.Parameterization.linearInterp(
            horizon_steps=T, num_control_points=nc
        )
        param = ampc.Parameterization.linearInterp(
            horizon_steps=T, endpoints=np.array([0, 1, 4])
        )

        param = ampc.Parameterization.bspline(
            horizon_steps=T, degree=2, num_control_points=nc
        )
        param = ampc.Parameterization.bspline(
            horizon_steps=T,
            degree=2,
            active_knots=np.array([0, 1, 4]),
        )

        param = ampc.Parameterization(horizon_steps=T, degree=1, num_control_points=nc)
        param = ampc.Parameterization(
            horizon_steps=T,
            degree=1,
            knots=np.array([0, 0, 1, 4, 4]),
        )

        knots = ampc.Parameterization.makeUniformClampedKnots(
            horizon_steps=T, degree=1, num_control_points=nc
        )
        assert isinstance(knots, np.ndarray)

    except:
        assert False


if __name__ == "__main__":
    test_parameterization_interface()
    print("All tests passed!")
