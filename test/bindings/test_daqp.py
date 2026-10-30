import os

import numpy as np
import pytest

import affine_mpc as ampc

requires_daqp = pytest.mark.skipif(
    not ampc.hasDAQP(), reason="affine_mpc was built without DAQP"
)


def _msd_mpc(formulation=ampc.Formulation.Condensed):
    mpc = ampc.MPC(
        2, 1, ampc.Parameterization.bspline(20, 2, 6), formulation=formulation
    )
    Ac = np.array([[0.0, 1.0], [-0.6, -0.1]])
    mpc.setModelContinuous2Discrete(Ac, np.array([0.0, 0.2]), np.zeros(2), 0.1)
    mpc.setInputLimits(np.zeros(1), 3.0 * np.ones(1))
    mpc.setReferenceState(np.array([1.0, 0.0]))
    return mpc


def test_wheels_are_built_with_daqp():
    # The wheel builds set this, so a wheel without DAQP fails instead of
    # skipping the tests below
    if os.environ.get("AFFINE_MPC_EXPECT_DAQP") == "1":
        assert ampc.hasDAQP()


def test_daqp_settings_defaults_and_fields():
    s = ampc.DAQPSettings()
    assert s.primal_tol == 1e-6
    assert s.dual_tol == 1e-12
    assert s.iter_limit == 10000
    assert s.time_limit == 0.0
    assert s.eps_prox == -1e-6
    s.iter_limit = 50
    s.primal_tol = 1e-7
    assert s.iter_limit == 50 and s.primal_tol == 1e-7
    assert repr(s).startswith("DAQPSettings(primal_tol=1e-07, ")


@pytest.mark.skipif(ampc.hasDAQP(), reason="affine_mpc was built with DAQP")
def test_initialize_with_daqp_settings_raises_without_daqp():
    with pytest.raises(RuntimeError, match="built without DAQP"):
        _msd_mpc().initializeSolver(ampc.DAQPSettings())


@requires_daqp
def test_daqp_matches_tight_osqp_in_closed_loop():
    daqp, ref = _msd_mpc(), _msd_mpc()
    assert daqp.initializeSolver(ampc.DAQPSettings())
    settings = ampc.OSQPSettings()
    settings.eps_abs = settings.eps_rel = 1e-10
    settings.max_iter = 100000
    settings.polishing = True
    assert ref.initializeSolver(settings)

    x = np.zeros(2)
    for _ in range(30):
        assert daqp.solve(x) == ampc.SolveStatus.Success
        assert ref.solve(x) == ampc.SolveStatus.Success
        info, ref_info = daqp.getSolveInfo(), ref.getSolveInfo()
        assert info.objective == pytest.approx(ref_info.objective, rel=1e-6, abs=1e-9)
        assert np.isnan(info.dual_residual)  # DAQP does not report residuals
        u = daqp.getNextInput()
        np.testing.assert_allclose(u, ref.getNextInput(), atol=1e-4)
        x = daqp.propagateModel(x, u)


@requires_daqp
def test_daqp_dual_solution_is_available():
    mpc = _msd_mpc()
    assert mpc.initializeSolver(ampc.DAQPSettings())
    assert mpc.solve(np.zeros(2)) == ampc.SolveStatus.Success
    y = mpc.getDualSolution()
    assert y.shape == (mpc.getNumQPConstraints(),)
    assert np.max(np.abs(y)) > 0.0  # the input starts at its upper limit


@requires_daqp
def test_daqp_rejects_sparse_formulation():
    with pytest.raises(ValueError):
        _msd_mpc(ampc.Formulation.Sparse).initializeSolver(ampc.DAQPSettings())
