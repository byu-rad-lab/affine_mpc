"""Tests of the deprecated names CondensedMPC, SparseMPC, and MPCBase (removed
in 0.5.0)."""

import subprocess
import sys
import warnings

import numpy as np
import pytest

import affine_mpc as ampc


def _solve_msd(mpc):
    Ac = np.array([[0.0, 1.0], [-0.6, -0.1]])
    mpc.setModelContinuous2Discrete(Ac, np.array([0.0, 0.2]), np.zeros(2), 0.1)
    mpc.setStateWeights(np.array([1.0, 0.1]))
    mpc.setReferenceState(np.array([1.0, 0.0]))
    mpc.setInputLimits(np.array([-3.0]), np.array([3.0]))
    assert mpc.initializeSolver()
    assert mpc.solve(np.zeros(2)) == ampc.SolveStatus.Success
    return mpc.getInputControlPoints()


@pytest.mark.parametrize(
    "name, formulation",
    [
        ("CondensedMPC", ampc.Formulation.Condensed),
        ("SparseMPC", ampc.Formulation.Sparse),
    ],
)
def test_deprecated_classes_warn_and_are_mpc(name, formulation):
    cls = getattr(ampc, name)
    param = ampc.Parameterization.linearInterp(10, 4)
    with pytest.warns(DeprecationWarning, match=f"{name} is deprecated.*use MPC"):
        old = cls(2, 1, param, ampc.Options(use_input_cost=True))
    assert isinstance(old, ampc.MPC)
    assert old.getFormulation() == formulation
    assert old.getOptions().use_input_cost
    with pytest.warns(DeprecationWarning):
        old_steps = cls(state_dim=2, input_dim=1, horizon_steps=10)
    assert old_steps.getFormulation() == formulation
    assert old_steps.num_control_points == 10

    with pytest.warns(DeprecationWarning):
        old = cls(2, 1, param)
    new = ampc.MPC(2, 1, param, formulation=formulation)
    np.testing.assert_array_equal(_solve_msd(old), _solve_msd(new))


def test_mpc_base_warns_and_is_mpc():
    with pytest.warns(DeprecationWarning, match="MPCBase is deprecated.*use MPC"):
        assert ampc.MPCBase is ampc.MPC
    with pytest.warns(DeprecationWarning):
        from affine_mpc import MPCBase  # noqa: F401


def test_import_does_not_warn():
    with warnings.catch_warnings():
        warnings.simplefilter("error")
        ampc.MPC(2, 1, 10)
    # a fresh interpreter, so the import itself runs (with the same sys.path)
    code = (
        "import sys; sys.path[:0] = %r; import affine_mpc; from affine_mpc import *"
        % (sys.path,)
    )
    result = subprocess.run(
        [sys.executable, "-W", "error::DeprecationWarning", "-c", code],
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stderr
