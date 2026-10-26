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


def test_parameterization_knot_ratio():
    import pytest

    T, g = 20, 1.5
    param = ampc.Parameterization.bspline(
        horizon_steps=T, degree=2, num_control_points=6, knot_ratio=g
    )
    lengths = g ** np.arange(4)
    active = np.concatenate([[0.0], np.cumsum(lengths)]) / lengths.sum() * (T - 1)
    np.testing.assert_allclose(param.knots[2:-2], active, rtol=0, atol=1e-12)

    uniform = ampc.Parameterization.moveBlocking(T, 4)
    np.testing.assert_array_equal(
        uniform.knots, ampc.Parameterization.makeUniformClampedKnots(T, 0, 4)
    )
    geo_mb = ampc.Parameterization.moveBlocking(T, 4, knot_ratio=0.5)
    assert geo_mb.degree == 0

    np.testing.assert_array_equal(
        ampc.Parameterization.makeClampedKnots(T, 2, 6, knot_ratio=g), param.knots
    )

    # a first interval below one sample is fine while the basis has full rank
    short = ampc.Parameterization.bspline(20, 1, 5, knot_ratio=3.0)
    assert short.knots[2] - short.knots[1] < 1.0
    with pytest.raises(ValueError, match="knot_ratio closer to 1"):
        ampc.Parameterization.bspline(20, 1, 5, knot_ratio=10.0)


def test_parameterization_rank_check():
    import pytest

    # a double knot is a valid spline: the input jumps at the knot
    jump = ampc.Parameterization.linearInterp(5, np.array([0.0, 2, 2, 4]))
    u = jump.evaluate(1, np.array([0.0, 2, 10, 6]))
    np.testing.assert_allclose(u, [0, 1, 10, 8, 6], rtol=0, atol=1e-15)

    # every basis function is nonzero at some sample, but control points 3 and
    # 4 are only nonzero at k = 4
    with pytest.raises(ValueError, match="control point 4 has no sample of its own"):
        ampc.Parameterization.bspline(10, 1, np.array([0, 2, 3.5, 3.9, 4.2, 4.6, 9]))


if __name__ == "__main__":
    test_parameterization_interface()
    print("All tests passed!")
