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


def test_parameterization_str_and_repr():
    param = ampc.Parameterization(horizon_steps=5, degree=1, num_control_points=3)
    assert (
        str(param)
        == "Parameterization(horizon_steps=5, degree=1, num_control_points=3, knots=[0, 0, 2, 4, 4])"
    )
    assert repr(param) == "Parameterization(horizon_steps=5, degree=1, num_control_points=3)"

    custom = ampc.Parameterization(
        horizon_steps=5,
        degree=1,
        knots=np.array([0, 0, 1, 4, 4]),
    )
    assert repr(custom) == "Parameterization(horizon_steps=5, degree=1, knots=[0, 0, 1, 4, 4])"


def test_parameterization_band_reproduces_evaluate():
    param = ampc.Parameterization.bspline(horizon_steps=12, degree=2, num_control_points=6)
    assert param.band_width == 3
    assert param.band_start.shape == (12,)
    assert param.band_weights.shape == (3, 12)

    m = 2
    ctrls = np.linspace(-1.0, 2.0, m * param.num_control_points).reshape(-1, m)
    u_traj = param.evaluate(m, ctrls.ravel()).reshape(-1, m)
    for k in range(param.horizon_steps):
        start = param.band_start[k]
        uk = param.band_weights[:, k] @ ctrls[start : start + param.band_width]
        np.testing.assert_allclose(u_traj[k], uk, rtol=0, atol=1e-15)


if __name__ == "__main__":
    test_parameterization_interface()
    print("All tests passed!")
