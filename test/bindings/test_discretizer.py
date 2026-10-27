import numpy as np
import pytest

import affine_mpc as ampc


def test_discretizer_matches_closed_form_double_integrator():
    disc = ampc.Discretizer(state_dim=2, input_dim=1)
    assert repr(disc) == "Discretizer(state_dim=2, input_dim=1)"
    assert not disc.hasDiscretized()

    dt = 0.05
    Ac = np.array([[0.0, 1.0], [0.0, 0.0]])
    Bc = np.array([[0.0], [1.0]])
    wc = np.array([0.0, -9.81])
    Ad, Bd, wd = disc.discretize(Ac, Bc, wc, dt)

    G = np.array([[dt, dt**2 / 2], [0.0, dt]])
    np.testing.assert_allclose(Ad, [[1.0, dt], [0.0, 1.0]], rtol=0, atol=1e-15)
    np.testing.assert_allclose(Bd, G @ Bc, rtol=0, atol=1e-15)
    np.testing.assert_allclose(wd, G @ wc, rtol=0, atol=1e-15)
    np.testing.assert_allclose(disc.getExponentialIntegral(), G, rtol=0, atol=1e-15)
    assert disc.hasDiscretized()

    # a new offset reuses G
    np.testing.assert_allclose(disc.discretizeOffset(np.array([1.0, 2.0])), G @ [1.0, 2.0])


def test_discretizer_matches_mpc_discretization():
    Ac = np.array([[0.0, 1.0], [-0.6, -0.1]])
    Bc = np.array([0.0, 0.2])
    wc = np.array([0.1, 0.0])
    Ad, Bd, wd = ampc.Discretizer(2, 1).discretize(Ac, Bc, wc, 0.1)

    mpc = ampc.MPC(2, 1, 10)
    mpc.setModelContinuous2Discrete(Ac, Bc, wc, 0.1)
    x, u = np.array([0.3, -0.2]), np.array([1.5])
    np.testing.assert_array_equal(mpc.propagateModel(x, u), Ad @ x + Bd @ u + wd)


def test_discretizer_errors():
    with pytest.raises(ValueError, match="state_dim must be positive"):
        ampc.Discretizer(0, 1)
    disc = ampc.Discretizer(2, 1)
    with pytest.raises(RuntimeError, match="discretize"):
        disc.discretizeOffset(np.zeros(2))
    with pytest.raises(ValueError, match="dt must be positive"):
        disc.discretize(np.eye(2), np.ones((2, 1)), np.zeros(2), 0.0)
