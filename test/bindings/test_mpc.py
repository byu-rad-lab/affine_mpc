import numpy as np
import pytest

import affine_mpc as ampc


def function_fails(func, *args, **kwargs):
    try:
        func(*args, **kwargs)
    except:
        return True
    return False


def test_mpc_interface():
    try:
        n, m, T, nc = 2, 1, 10, 5
        param = ampc.Parameterization.linearInterp(
            horizon_steps=T, num_control_points=nc
        )
        opts = ampc.Options()
        opts.use_input_cost = True
        opts.slew_control_points = True
        opts.slew_initial_input = True
        opts.state_bounds = ampc.StateBoundMode.Hard
        mpc = ampc.MPC(n, m, param, opts)

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
        == "MPC(formulation=Condensed, state_dim=2, input_dim=1, parameterization=Parameterization(horizon_steps=10, degree=1, num_control_points=5, knots=[0, 0, 2.25, 4.5, 6.75, 9, 9]), options=Options(use_input_cost=True, use_input_rate_cost=False, saturate_input_trajectory=False, slew_initial_input=True, slew_control_points=True, state_bounds=Hard, bounded_states=[], num_general_constraints=0, num_terminal_constraints=0), solver_initialized=False, Q=[1, 1], Qf=[1, 1], R=[1], u_min=[-1], u_max=[1], x_min=[-1, -1], x_max=[1, 1], u0_slew=[1], control_point_slew=[1])"
    )
    assert str(mpc).startswith("MPC:\n  formulation = Condensed\n  state_dim = 2\n  input_dim = 1\n")

    assert function_fails(mpc.initializeSolver, solver_settings=None)
    # the default formulation is condensed
    assert mpc.initializeSolver(solver_settings=ampc.OSQPSettings())

    assert function_fails(mpc.getNextInput, u0=None)
    assert function_fails(mpc.getInputControlPoints, control_points=None)
    assert function_fails(mpc.getInputTrajectory, u_traj=None)
    assert function_fails(mpc.getPredictedStateTrajectory, x_traj=None)


if __name__ == "__main__":
    test_mpc_interface()
    print("All tests passed!")


def _all_options_mpc(n=2, m=2, T=6, nc=3):
    param = ampc.Parameterization.linearInterp(horizon_steps=T, num_control_points=nc)
    opts = ampc.Options(
        use_input_cost=True,
        slew_initial_input=True,
        slew_control_points=True,
        state_bounds=ampc.StateBoundMode.Hard,
    )
    return ampc.MPC(n, m, param, opts), param, opts


def test_mpc_getters_return_set_values():
    n, m, T, nc = 2, 2, 6, 3
    mpc, param, opts = _all_options_mpc(n, m, T, nc)

    assert str(mpc.getOptions()) == str(opts)
    stored = mpc.getParameterization()
    assert stored.num_control_points == nc
    np.testing.assert_array_equal(stored.knots, param.knots)
    np.testing.assert_array_equal(stored.band_weights, param.band_weights)
    assert np.all(mpc.getInputMin() == -np.inf)
    assert np.all(mpc.getSlewRate() == np.inf)

    Q, Qf, R = np.array([1.0, 2.0]), np.array([3.0, 4.0]), np.array([0.5, 0.6])
    mpc.setWeights(Q_diag=Q, Qf_diag=Qf, R_diag=R)
    np.testing.assert_array_equal(mpc.getStateWeights(), Q)
    # the 3-argument setWeights used to drop Qf_diag
    np.testing.assert_array_equal(mpc.getTerminalStateWeights(), Qf)
    np.testing.assert_array_equal(mpc.getInputWeights(), R)

    mpc.setInputLimits(u_min=-np.ones(m), u_max=np.ones(m))
    mpc.setStateLimits(x_min=-5 * np.ones(n), x_max=5 * np.ones(n))
    mpc.setSlewRate(control_point_slew=np.array([0.1, 0.2]))
    mpc.setSlewRateInitial(u0_slew=np.array([0.3, 0.4]))
    mpc.setPreviousInput(u_prev=np.array([0.7, 0.8]))
    np.testing.assert_array_equal(mpc.getInputMin(), -np.ones(m))
    np.testing.assert_array_equal(mpc.getInputMax(), np.ones(m))
    np.testing.assert_array_equal(mpc.getStateMin(), -5 * np.ones(n))
    np.testing.assert_array_equal(mpc.getStateMax(), 5 * np.ones(n))
    np.testing.assert_array_equal(mpc.getSlewRate(), [0.1, 0.2])
    np.testing.assert_array_equal(mpc.getSlewRateInitial(), [0.3, 0.4])
    np.testing.assert_array_equal(mpc.getPreviousInput(), [0.7, 0.8])

    x_ref = np.linspace(0, 1, n * T)
    ctrls_ref = np.linspace(-1, 1, m * nc)
    mpc.setReferenceStateTrajectory(x_traj=x_ref)
    mpc.setReferenceInputControlPoints(control_points=ctrls_ref)
    np.testing.assert_array_equal(mpc.getReferenceStateTrajectory(), x_ref)
    np.testing.assert_array_equal(mpc.getReferenceInputControlPoints(), ctrls_ref)

    # getters return copies, so editing one does not change the MPC
    u_min = mpc.getInputMin()
    u_min[:] = 0.0
    np.testing.assert_array_equal(mpc.getInputMin(), -np.ones(m))


def test_mpc_optional_getters_raise_when_disabled():
    mpc = ampc.MPC(2, 1, ampc.Parameterization.linearInterp(5, 3))
    for getter in (
        mpc.getInputWeights,
        mpc.getReferenceInputControlPoints,
        mpc.getStateMin,
        mpc.getStateMax,
        mpc.getSlewRate,
        mpc.getSlewRateInitial,
        mpc.getPreviousInput,
    ):
        with pytest.raises(RuntimeError, match="not enabled"):
            getter()


def _msd_mpc(formulation):
    n, m, T = 2, 1, 20
    mpc = ampc.MPC(
        n, m, ampc.Parameterization.linearInterp(T, 5), formulation=formulation
    )
    Ac = np.array([[0.0, 1.0], [-0.6, -0.1]])
    mpc.setModelContinuous2Discrete(Ac, np.array([0.0, 0.2]), np.zeros(n), 0.1)
    mpc.setInputLimits(np.zeros(m), 3.0 * np.ones(m))
    mpc.setReferenceState(np.array([1.0, 0.0]))
    assert mpc.initializeSolver()
    return mpc


@pytest.mark.parametrize("formulation", list(ampc.Formulation))
def test_solve_in_threads_matches_sequential_solve(formulation):
    # solve() releases the GIL, so independent MPC objects can solve in parallel
    from concurrent.futures import ThreadPoolExecutor

    x0s = [np.array([0.1 * i, -0.05 * i]) for i in range(8)]

    def solve(x0):
        mpc = _msd_mpc(formulation)
        assert mpc.solve(x0) == ampc.SolveStatus.Success
        return mpc.getInputTrajectory()

    expected = [solve(x0) for x0 in x0s]
    with ThreadPoolExecutor(max_workers=4) as pool:
        got = list(pool.map(solve, x0s))
    for u_got, u_expected in zip(got, expected):
        np.testing.assert_array_equal(u_got, u_expected)

    # exceptions still propagate with the GIL released during the call
    with pytest.raises(ValueError, match="x0"):
        _msd_mpc(formulation).solve(np.zeros(3))


@pytest.mark.parametrize("formulation", list(ampc.Formulation))
def test_model_offset_setters(formulation):
    n, m = 2, 1
    Ac = np.array([[0.0, 1.0], [-0.6, -0.1]])
    Bc = np.array([0.0, 0.2])
    x, u = np.array([0.5, -0.2]), np.array([1.5])

    mpc = ampc.MPC(
        n, m, ampc.Parameterization.linearInterp(20, 5), formulation=formulation
    )
    with pytest.raises(RuntimeError, match="Model must be set"):
        mpc.setModelOffsetDiscrete(wd=np.zeros(n))

    mpc.setModelContinuous2Discrete(Ac, Bc, np.zeros(n), 0.1)
    wc = np.array([0.3, -0.4])
    mpc.setModelOffsetContinuous2Discrete(wc=wc)
    Ad, Bd, wd = ampc.Discretizer(n, m).discretize(Ac, Bc, wc, 0.1)
    np.testing.assert_allclose(mpc.propagateModel(x, u), Ad @ x + Bd @ u + wd, atol=1e-15)

    mpc.setModelOffsetDiscrete(wd=np.array([0.01, 0.02]))
    np.testing.assert_allclose(
        mpc.propagateModel(x, u), Ad @ x + Bd @ u + [0.01, 0.02], atol=1e-15
    )

    mpc.setModelDiscrete(Ad, Bd, wd)
    with pytest.raises(RuntimeError, match="setModelContinuous2Discrete"):
        mpc.setModelOffsetContinuous2Discrete(wc)


@pytest.mark.parametrize("formulation", list(ampc.Formulation))
def test_input_rate_cost(formulation):
    n, m, T = 2, 1, 20
    param = ampc.Parameterization.bspline(T, 2, 8)

    def solve(rate_weight):
        mpc = ampc.MPC(
            n,
            m,
            param,
            ampc.Options(use_input_rate_cost=True),
            formulation=formulation,
        )
        mpc.setModelContinuous2Discrete(
            np.array([[0.0, 1.0], [-0.6, -0.1]]), np.array([0.0, 0.2]), np.zeros(n), 0.1
        )
        mpc.setInputLimits(u_min=-np.ones(m), u_max=np.ones(m))
        mpc.setReferenceState(np.array([1.0, 0.0]))
        mpc.setInputRateWeights(R_rate_diag=np.full(m, rate_weight))
        np.testing.assert_array_equal(mpc.getInputRateWeights(), [rate_weight])
        assert mpc.initializeSolver()
        assert mpc.solve(np.zeros(n)) == ampc.SolveStatus.Success
        # the previous input follows the solves without slew_initial_input
        np.testing.assert_array_equal(mpc.getPreviousInput(), mpc.getNextInput())
        assert "R_rate=" in repr(mpc)
        return mpc.getInputTrajectory()

    def variation(u):
        return np.abs(np.diff(np.concatenate(([0.0], u)))).sum()

    assert variation(solve(100.0)) < 0.5 * variation(solve(0.01))

    with pytest.raises(RuntimeError, match="not enabled"):
        ampc.MPC(n, m, param, formulation=formulation).setInputRateWeights(np.ones(m))


@pytest.mark.parametrize("formulation", list(ampc.Formulation))
def test_partial_and_soft_state_bounds(formulation):
    param = ampc.Parameterization.bspline(20, 2, 8)
    settings = ampc.OSQPSettings()
    settings.polishing = True
    settings.max_iter = 20000

    def build(opts, x0):
        mpc = ampc.MPC(2, 1, param, opts, formulation=formulation)
        mpc.setModelContinuous2Discrete(
            np.array([[0.0, 1.0], [-0.6, -0.1]]), np.array([0.0, 0.2]), np.zeros(2), 0.1
        )
        mpc.setStateWeights(np.array([1.0, 0.1]))
        mpc.setReferenceState(np.array([1.0, 0.0]))
        mpc.setInputLimits(u_min=np.array([-3.0]), u_max=np.array([3.0]))
        mpc.setStateLimits(x_min=np.array([-0.5]), x_max=np.array([0.5]))
        if opts.state_bounds == ampc.StateBoundMode.Soft:
            mpc.setStateBoundPenalties(l1=np.array([10.0]), l2=np.array([100.0]))
        assert mpc.initializeSolver(settings)
        return mpc, mpc.solve(x0)

    opts = ampc.Options(state_bounds=ampc.StateBoundMode.Hard, bounded_states=[0])
    assert opts.bounded_states == [0]
    assert "bounded_states=[0]" in repr(opts)
    hard, status = build(opts, np.zeros(2))
    assert status == ampc.SolveStatus.Success
    assert hard.getBoundedStates() == [0]
    x_traj = hard.getPredictedStateTrajectory().reshape(-1, 2)
    assert x_traj[:, 0].max() == pytest.approx(0.5, abs=1e-7)

    soft_opts = ampc.Options(state_bounds=ampc.StateBoundMode.Soft, bounded_states=[0])
    soft, status = build(soft_opts, np.zeros(2))
    assert status == ampc.SolveStatus.Success
    np.testing.assert_allclose(
        soft.getInputControlPoints(), hard.getInputControlPoints(), atol=1e-7
    )
    np.testing.assert_array_equal(soft.getStateBoundL2Penalties(), [100.0])

    # starting past the limit: infeasible when hard, solvable when soft
    _, status = build(opts, np.array([0.9, 0.0]))
    assert status == ampc.SolveStatus.PrimalInfeasible
    _, status = build(soft_opts, np.array([0.9, 0.0]))
    assert status == ampc.SolveStatus.Success

    with pytest.raises(ValueError, match="strictly increasing"):
        ampc.MPC(
            2,
            1,
            param,
            ampc.Options(state_bounds=ampc.StateBoundMode.Hard, bounded_states=[1, 0]),
            formulation=formulation,
        )


@pytest.mark.parametrize("formulation", list(ampc.Formulation))
def test_general_and_terminal_constraints(formulation):
    param = ampc.Parameterization.bspline(20, 2, 8)
    T = param.horizon_steps
    settings = ampc.OSQPSettings()
    settings.polishing = True
    settings.max_iter = 20000

    opts = ampc.Options(num_general_constraints=1, num_terminal_constraints=1)
    assert opts.num_general_constraints == 1
    assert "num_general_constraints=1, num_terminal_constraints=1" in repr(opts)
    mpc = ampc.MPC(2, 1, param, opts, formulation=formulation)
    mpc.setModelContinuous2Discrete(
        np.array([[0.0, 1.0], [-0.6, -0.1]]), np.array([0.0, 0.2]), np.zeros(2), 0.1
    )
    mpc.setStateWeights(np.array([1.0, 0.1]))
    mpc.setReferenceState(np.array([1.0, 0.0]))
    mpc.setInputLimits(u_min=np.array([-3.0]), u_max=np.array([3.0]))
    # x_k + u_k <= 3 at every step, x_T <= 0.8 at the end
    Hx, Hu = np.array([[1.0, 0.0]]), np.array([[1.0]])
    mpc.setGeneralConstraints(
        Hx=Hx, Hu=Hu, h_min=np.array([-np.inf]), h_max=np.array([3.0])
    )
    mpc.setTerminalConstraints(
        Hf=np.array([[1.0, 0.0]]), hf_min=np.array([-np.inf]), hf_max=np.array([0.8])
    )
    assert mpc.initializeSolver(settings)
    x0 = np.zeros(2)
    assert mpc.solve(x0) == ampc.SolveStatus.Success

    x = mpc.getPredictedStateTrajectory().reshape(T, 2)
    u = mpc.getInputTrajectory()
    g = np.concatenate([[x0[0] + u[0]], x[:-1, 0] + u[1:]])
    assert g.max() <= 3.0 + 1e-7
    assert x[-1, 0] <= 0.8 + 1e-7

    np.testing.assert_array_equal(mpc.getGeneralConstraintStateMatrix(), Hx)
    np.testing.assert_array_equal(mpc.getGeneralConstraintInputMatrix(), Hu)
    np.testing.assert_array_equal(mpc.getGeneralConstraintMax(), [3.0])
    np.testing.assert_array_equal(mpc.getTerminalConstraintMax(), [0.8])
    assert "Hx=[[1, 0]]" in repr(mpc)

    with pytest.raises(ValueError, match="cannot be greater"):
        mpc.setGeneralConstraints(Hx, Hu, np.array([1.0]), np.array([0.0]))
    with pytest.raises(RuntimeError, match="not enabled"):
        ampc.MPC(2, 1, param, formulation=formulation).setGeneralConstraints(
            Hx, Hu, np.zeros(1), np.ones(1)
        )
    with pytest.raises(RuntimeError, match="not enabled"):
        ampc.MPC(2, 1, param, formulation=formulation).getTerminalConstraintMatrix()
    with pytest.raises(ValueError, match="non-negative"):
        ampc.MPC(
            2,
            1,
            param,
            ampc.Options(num_general_constraints=-1),
            formulation=formulation,
        )


@pytest.mark.parametrize("formulation", list(ampc.Formulation))
def test_qp_blocks(formulation):
    mpc = _msd_mpc(formulation)
    sparse = formulation == ampc.Formulation.Sparse

    variables = mpc.getQPVariableBlocks()
    assert [b.name for b in variables] == ["control_points"] + (
        ["states"] if sparse else []
    )
    assert (variables[0].offset, variables[0].size) == (0, 5)
    assert repr(variables[0]) == "QPBlock(name='control_points', offset=0, size=5)"
    assert sum(b.size for b in variables) == mpc.getNumQPVariables()

    rows = mpc.getQPConstraintBlocks()
    assert [b.name for b in rows] == (["dynamics"] if sparse else []) + ["input_bounds"]
    assert sum(b.size for b in rows) == mpc.getNumQPConstraints()


@pytest.mark.parametrize("formulation", list(ampc.Formulation))
def test_get_and_assemble_qp(formulation):
    scipy_sparse = pytest.importorskip("scipy.sparse")
    mpc = _msd_mpc(formulation)
    x0 = np.array([0.2, 0.0])
    assert mpc.solve(x0) == ampc.SolveStatus.Success

    qp = mpc.getQP()
    nv, nr = mpc.getNumQPVariables(), mpc.getNumQPConstraints()
    for M in (qp.P, qp.A):
        assert scipy_sparse.issparse(M) and M.format == "csc"
    assert qp.P.shape == (nv, nv) and qp.A.shape == (nr, nv)
    assert qp.q.shape == (nv,) and qp.l.shape == (nr,) and qp.u.shape == (nr,)
    assert abs(qp.P - qp.P.T).max() == 0.0

    # the solution, in the order of the variable blocks, gives the solver's
    # objective and satisfies the constraints
    z = np.empty(nv)
    for b in mpc.getQPVariableBlocks():
        if b.name == "control_points":
            z[b.offset : b.offset + b.size] = mpc.getInputControlPoints()
        else:
            z[b.offset : b.offset + b.size] = mpc.getPredictedStateTrajectory()
    objective = 0.5 * z @ (qp.P @ z) + qp.q @ z
    assert objective == pytest.approx(mpc.getSolveInfo().objective, rel=1e-9, abs=1e-12)
    Az = qp.A @ z
    assert np.all(Az >= qp.l - 1e-5) and np.all(Az <= qp.u + 1e-5)

    # assembleQP() fills the given object with the same QP for the same x0
    out = ampc.QuadraticProgram()
    mpc.assembleQP(x0, out)
    assert abs(out.P - qp.P).max() == 0.0 and abs(out.A - qp.A).max() == 0.0
    for a, b in ((out.q, qp.q), (out.l, qp.l), (out.u, qp.u)):
        np.testing.assert_array_equal(a, b)


def test_assemble_qp_requires_setup():
    mpc = ampc.MPC(2, 1, ampc.Parameterization.linearInterp(10, 4))
    with pytest.raises(
        RuntimeError, match="Model must be set before assembling the QP"
    ):
        mpc.assembleQP(np.zeros(2), ampc.QuadraticProgram())


def test_mpc_formulation_argument():
    Sparse, Condensed = ampc.Formulation.Sparse, ampc.Formulation.Condensed
    param = ampc.Parameterization.linearInterp(10, 4)
    opts = ampc.Options(use_input_cost=True)
    cases = [
        (ampc.MPC(2, 1, param), Condensed, False, 4),
        (ampc.MPC(2, 1, param, opts), Condensed, True, 4),
        (ampc.MPC(2, 1, param, opts, formulation=Sparse), Sparse, True, 4),
        (ampc.MPC(2, 1, param, formulation=Sparse), Sparse, False, 4),
        (ampc.MPC(state_dim=2, input_dim=1, horizon_steps=10), Condensed, False, 10),
        (ampc.MPC(2, 1, 10, opts=opts, formulation=Sparse), Sparse, True, 10),
    ]
    for mpc, formulation, input_cost, nc in cases:
        assert mpc.getFormulation() == formulation
        assert mpc.getOptions().use_input_cost == input_cost
        assert mpc.num_control_points == nc
        names = [b.name for b in mpc.getQPVariableBlocks()]
        assert ("states" in names) == (formulation == Sparse)
        assert str(mpc).startswith(f"MPC:\n  formulation = {formulation.name}\n")
        assert repr(mpc).startswith(f"MPC(formulation={formulation.name}, ")

    # the formulation is keyword-only, so it can not be taken for the options
    with pytest.raises(TypeError):
        ampc.MPC(2, 1, param, opts, Sparse)


def test_mpc_logger_records_formulation(tmp_path):
    param = ampc.Parameterization.linearInterp(5, 2)
    for formulation in (ampc.Formulation.Condensed, ampc.Formulation.Sparse):
        mpc = ampc.MPC(2, 1, param, formulation=formulation)
        logger = ampc.MPCLogger(mpc, tmp_path / formulation.name, 0.1)
        logger.finalize()
        params = (tmp_path / formulation.name / "params.yaml").read_text()
        assert f"formulation: {formulation.name.lower()}\n" in params
