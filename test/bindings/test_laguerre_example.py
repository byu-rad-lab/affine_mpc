"""Checks of the Laguerre basis and the deviation form in examples/laguerre.py."""

import importlib.util
from pathlib import Path

import numpy as np
import pytest

EXAMPLE = Path(__file__).resolve().parents[2] / "examples" / "laguerre.py"


@pytest.fixture(scope="module")
def example():
    if not EXAMPLE.exists():
        pytest.skip("examples/laguerre.py not found")
    spec = importlib.util.spec_from_file_location("laguerre_example", EXAMPLE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_laguerre_basis_matches_closed_forms(example):
    T, N, a = 20, 4, 0.6
    beta = 1 - a**2
    basis = example.laguerre_basis(T, N, a)
    assert basis.shape == (T, N)
    np.testing.assert_allclose(basis[:, 0], np.sqrt(beta) * a ** np.arange(T), rtol=0, atol=1e-14)
    np.testing.assert_allclose(basis[0], np.sqrt(beta) * (-a) ** np.arange(N), rtol=0, atol=1e-14)

    # pole 0: unit pulses, so inputs after the first N steps are zero
    np.testing.assert_array_equal(example.laguerre_basis(8, 3, 0.0), np.eye(8, 3))


def test_laguerre_basis_is_orthonormal_only_over_long_horizons(example):
    # orthonormal over an infinite horizon; the truncation error decays like
    # a^(2T)
    N, a = 5, 0.6
    long_basis = example.laguerre_basis(400, N, a)
    np.testing.assert_allclose(long_basis.T @ long_basis, np.eye(N), rtol=0, atol=1e-10)
    short_basis = example.laguerre_basis(10, N, a)
    assert np.abs(short_basis.T @ short_basis - np.eye(N)).max() > 1e-2


def test_prepared_basis_is_orthonormal_with_the_same_span(example):
    T, N, a = 40, 6, 0.9
    basis = example.laguerre_basis(T, N, a)
    q = example.prepare_basis(basis)
    np.testing.assert_allclose(q.T @ q, np.eye(N), rtol=0, atol=1e-12)
    # least squares fit of the original columns in the new basis is exact
    coeffs = np.linalg.lstsq(q, basis, rcond=None)[0]
    np.testing.assert_allclose(q @ coeffs, basis, rtol=0, atol=1e-12)

    # rows decayed to round-off stay negligible after QR, so the MPC omits
    # their input bounds
    decayed = example.prepare_basis(example.laguerre_basis(300, 3, 0.5))
    norms = np.linalg.norm(decayed, axis=1)
    assert norms[-50:].max() <= 1e-9 * norms.max()
    mpc = example.ampc.MPC(2, 1, example.ampc.Parameterization.fromBasis(300, decayed))
    rows = {b.name: b.size for b in mpc.getQPConstraintBlocks()}
    assert rows["input_bounds"] == np.count_nonzero(norms > 1e-9 * norms.max()) < 250


def test_deviation_form_holds_the_reference(example):
    direct_position, direct_cost = example.run(deviation=False)
    deviation_position, deviation_cost = example.run(deviation=True)
    assert abs(deviation_position - 1.0) < 1e-3
    # the direct form decays to zero over the horizon and settles short
    assert abs(direct_position - 1.0) > 0.05
    assert deviation_cost < direct_cost
