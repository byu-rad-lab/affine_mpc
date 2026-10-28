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


def _laguerre_basis(horizon_steps, num_functions, pole):
    """Discrete Laguerre functions (Wang, 2004), one per column."""
    beta = 1 - pole**2
    omega = np.zeros((num_functions, num_functions))
    for i in range(num_functions):
        for j in range(i + 1):
            omega[i, j] = pole if i == j else beta * (-pole) ** (i - j - 1)
    L = np.sqrt(beta) * (-pole) ** np.arange(num_functions)
    basis = np.zeros((horizon_steps, num_functions))
    for k in range(horizon_steps):
        basis[k] = L
        L = omega @ L
    return basis


def test_from_basis_interface():
    import pytest

    bspline = ampc.Parameterization.bspline(horizon_steps=10, degree=2, num_control_points=5)
    assert bspline.type == ampc.Parameterization.Type.BSpline
    basis = np.zeros((10, 5))
    for k in range(10):
        start = bspline.band_start[k]
        basis[k, start : start + bspline.band_width] = bspline.band_weights[:, k]

    param = ampc.Parameterization.fromBasis(horizon_steps=10, basis=basis)
    assert param.type == ampc.Parameterization.Type.Custom
    assert param.horizon_steps == 10
    assert param.num_control_points == 5
    assert param.degree == -1
    assert param.knots.size == 0
    assert param.band_width == 5
    np.testing.assert_array_equal(param.band_start, np.zeros(10))
    np.testing.assert_array_equal(param.band_weights, basis.T)
    ctrls = np.linspace(-1.0, 2.0, 10)
    np.testing.assert_allclose(param.evaluate(2, ctrls), bspline.evaluate(2, ctrls), rtol=0, atol=1e-15)

    expected = "Parameterization(type=custom, horizon_steps=10, num_control_points=5)"
    assert str(param) == expected
    assert repr(param) == expected

    with pytest.raises(ValueError, match="full column rank"):
        ampc.Parameterization.fromBasis(4, np.ones((4, 2)))
    with pytest.raises(ValueError, match="finite"):
        ampc.Parameterization.fromBasis(4, np.full((4, 2), np.nan))
    with pytest.raises(ValueError, match="rows"):
        ampc.Parameterization.fromBasis(5, np.eye(4, 2))


def _custom_basis_mpc(formulation, param, opts):
    mpc = ampc.MPC(2, 1, param, opts, formulation=formulation)
    A = np.array([[0.0, 1.0], [-0.6, -0.1]])
    mpc.setModelContinuous2Discrete(A, np.array([0.0, 0.2]), np.zeros(2), 0.1)
    mpc.setInputLimits(np.array([0.0]), np.array([3.0]))
    mpc.setWeights(np.array([1.0, 0.1]), np.array([1e-3]))
    mpc.setReferenceState(np.array([1.0, 0.0]))
    settings = ampc.OSQPSettings()
    settings.polishing = 1
    settings.max_iter = 20000
    assert mpc.initializeSolver(settings)
    return mpc


def test_from_basis_mpc_bounds_sampled_inputs():
    import pytest

    T = 30
    param = ampc.Parameterization.fromBasis(T, _laguerre_basis(T, 4, 0.8))
    for formulation in list(ampc.Formulation):
        mpc = _custom_basis_mpc(formulation, param, ampc.Options(input_cost=ampc.InputCostMode.ControlPoints))
        assert mpc.solve(np.array([0.5, -0.2])) == ampc.SolveStatus.Success
        u_traj = mpc.getInputTrajectory()
        assert u_traj.min() >= -1e-6
        assert u_traj.max() <= 3.0 + 1e-6
        np.testing.assert_allclose(u_traj.max(), 3.0, atol=1e-6)

        with pytest.raises(RuntimeError, match="custom basis"):
            mpc.setReferenceInput(np.zeros(1))

    with pytest.raises(ValueError, match="slew_control_points requires a B-spline"):
        ampc.MPC(2, 1, param, ampc.Options(slew_control_points=True))


def test_from_basis_logger_metadata(tmp_path):
    T = 12
    basis = _laguerre_basis(T, 3, 0.6)
    param = ampc.Parameterization.fromBasis(T, basis)
    mpc = _custom_basis_mpc(
        ampc.Formulation.Condensed, param, ampc.Options(input_cost=ampc.InputCostMode.ControlPoints)
    )
    logger = ampc.MPCLogger(
        mpc, tmp_path, 0.1, log_control_points=True, mode=ampc.MPCLogger.Mode.NpzUncompressed
    )
    x0 = np.array([0.5, -0.2])
    assert mpc.solve(x0) == ampc.SolveStatus.Success
    logger.logStep(0.0, x0)
    logger.finalize()

    with np.load(tmp_path / "log.npz") as data:
        assert "meta_knots" not in data.files
        assert "meta_spline_degree" not in data.files
        logged_basis = data["meta_basis"].reshape(T, 3)
        np.testing.assert_array_equal(logged_basis, basis)
        # the logged control points are coefficients of the logged basis
        u_traj = logged_basis @ data["inputs"][0].reshape(-1)
        np.testing.assert_allclose(u_traj, mpc.getInputTrajectory(), rtol=0, atol=1e-12)
    params = (tmp_path / "params.yaml").read_text()
    assert "parameterization_type: custom" in params


if __name__ == "__main__":
    test_parameterization_interface()
    print("All tests passed!")
