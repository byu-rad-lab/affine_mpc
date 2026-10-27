import numpy as np
import pytest

import affine_mpc as ampc

N, M, T, NC = 4, 2, 12, 5
X0 = np.array([0.3, -0.1, 0.2, 0.05])


def model(tilt):
    """Two double integrators with a weak cross term; the entries scaled by
    tilt are zero at tilt = 0."""
    Ad = np.array(
        [
            [1.0, 0.1, 0.0, 0.0],
            [0.0, 1.0, 0.0, tilt * 0.2],
            [0.0, 0.0, 1.0, 0.1],
            [0.0, 0.0, tilt * 0.3, 0.98],
        ]
    )
    Bd = np.array([[0.005, 0.0], [0.1, tilt * 0.05], [0.0, 0.005], [tilt * 0.02, 0.1]])
    return Ad, Bd


def build(formulation, init_tilt, **init_kwargs):
    opts = ampc.Options(use_input_cost=True, saturate_states=True)
    mpc = ampc.MPC(
        N, M, ampc.Parameterization.bspline(T, 2, NC), opts, formulation=formulation
    )
    mpc.setModelDiscrete(*model(init_tilt), np.zeros(N))
    mpc.setInputLimits(-np.ones(M), np.ones(M))
    mpc.setStateLimits(np.full(N, -3.0), np.full(N, 3.0))
    mpc.setWeights(np.array([10.0, 1.0, 10.0, 1.0]), np.full(M, 0.1))
    mpc.setReferenceState(np.array([1.0, 0.0, -0.5, 0.0]))
    settings = ampc.OSQPSettings()
    settings.eps_abs = settings.eps_rel = 1e-9
    settings.max_iter = 20000
    assert mpc.initializeSolver(settings, **init_kwargs)
    return mpc


@pytest.mark.parametrize(
    "formulation, init_tilt, kwargs",
    [
        # a representative model at initialization tracks every later entry
        (ampc.Formulation.Condensed, 1.0, {}),
        (ampc.Formulation.Sparse, 1.0, {}),
        # Full tracks entries that are zero at initialization
        (ampc.Formulation.Sparse, 0.0, {"model_sparsity": ampc.ModelSparsity.Full}),
    ],
)
def test_matches_full(formulation, init_tilt, kwargs):
    mpc = build(formulation, init_tilt, **kwargs)
    full = build(
        ampc.Formulation.Sparse, init_tilt, model_sparsity=ampc.ModelSparsity.Full
    )
    for tilt in (0.0, 0.5, -1.0, 1.0):
        for each in (mpc, full):
            each.setModelDiscrete(*model(tilt), np.zeros(N))
            each.resetWarmStart()
            assert each.solve(X0) == ampc.SolveStatus.Success
        assert mpc.isWithinSparsityPattern()
        np.testing.assert_allclose(
            mpc.getInputControlPoints(), full.getInputControlPoints(), atol=1e-6
        )


@pytest.mark.parametrize("formulation", list(ampc.Formulation))
def test_initial_nonzeros_reports_later_entries(formulation):
    # the library is usually built without asserts (Release)
    mpc = build(formulation, 0.0)
    assert mpc.getModelSparsity() == ampc.ModelSparsity.InitialNonzeros
    mpc.setModelDiscrete(*model(1.0), np.zeros(N))
    mpc.solve(X0)
    assert not mpc.isWithinSparsityPattern()


def test_sparse_initial_nonzeros_tracks_fewer_entries():
    pytest.importorskip("scipy")
    # initialized at tilt = 0, then set to tilt = 1
    tracked = build(ampc.Formulation.Sparse, 0.0)
    full = build(ampc.Formulation.Sparse, 0.0, model_sparsity=ampc.ModelSparsity.Full)
    Ad, Bd = model(1.0)
    for mpc in (tracked, full):
        mpc.setModelDiscrete(Ad, Bd, np.zeros(N))
        assert mpc.prepare() == ampc.SolveStatus.Success
    # the tilt entries (2 of Ad, 2 of Bd) are not tracked: Ad on the states of
    # steps 1 to T-1, Bd times each nonzero basis weight
    weights = ampc.Parameterization.bspline(T, 2, NC).band_weights
    expected = 2 * (T - 1) + 2 * int(np.count_nonzero(weights))
    assert full.getQP().A.nnz - tracked.getQP().A.nnz == expected


def test_condensed_full_raises():
    with pytest.raises(ValueError):
        build(ampc.Formulation.Condensed, 1.0, model_sparsity=ampc.ModelSparsity.Full)
