import affine_mpc as ampc


def test_solve_status_interface():
    try:
        s = ampc.SolveStatus(value=0)
        s = ampc.SolveStatus.Success
        s = ampc.SolveStatus.NotInitialized
        s = ampc.SolveStatus.SolvedInaccurate
        s = ampc.SolveStatus.PrimalInfeasible
        s = ampc.SolveStatus.DualInfeasible
        s = ampc.SolveStatus.MaxIterReached
        s = ampc.SolveStatus.TimeLimitReached
        s = ampc.SolveStatus.OtherFailure
        s = ampc.SolveStatus.UpdateFailed
    except:
        assert False


if __name__ == "__main__":
    test_solve_status_interface()


def test_solve_info_from_mpc():
    import numpy as np

    mpc = ampc.MPC(2, 1, 10)
    assert mpc.getSolveInfo().status == ampc.SolveStatus.NotInitialized

    mpc.setModelDiscrete(
        np.array([[1.0, 0.1], [-0.06, 0.99]]), np.array([0.0, 0.02]), np.zeros(2)
    )
    mpc.setInputLimits(np.array([-1.0]), np.array([1.0]))
    mpc.setReferenceState(np.array([1.0, 0.0]))
    assert mpc.initializeSolver()
    assert mpc.solve(np.zeros(2)) == ampc.SolveStatus.Success

    info = mpc.getSolveInfo()
    assert isinstance(info, ampc.SolveInfo)
    assert info.status == ampc.SolveStatus.Success
    assert info.iterations > 0
    assert np.isfinite(info.objective)
    assert info.mpc_solve_time > 0.0
    assert info.mpc_solve_time >= info.qp_solve_time >= 0.0
    assert not hasattr(info, "run_time")
    assert repr(info).startswith("SolveInfo(status=Success, iterations=")
    assert info.prepare_time == 0.0  # prepare() was never called


def _configured_mpc(formulation):
    import numpy as np

    mpc = ampc.MPC(
        2, 1, ampc.Parameterization.bspline(10, 2, 5), formulation=formulation
    )
    mpc.setModelDiscrete(
        np.array([[1.0, 0.1], [-0.06, 0.99]]), np.array([0.0, 0.02]), np.zeros(2)
    )
    mpc.setInputLimits(np.array([-1.0]), np.array([1.0]))
    mpc.setReferenceState(np.array([1.0, 0.0]))
    return mpc


def test_prepare_then_solve_matches_solve_alone():
    import numpy as np

    for formulation in ampc.Formulation:
        prepared, direct = _configured_mpc(formulation), _configured_mpc(formulation)
        assert prepared.prepare() == ampc.SolveStatus.NotInitialized
        assert prepared.initializeSolver() and direct.initializeSolver()

        x0 = np.array([0.3, -0.1])
        for scale in (1.0, 1.01, 1.02):
            for mpc in (prepared, direct):
                mpc.setModelDiscrete(
                    scale * np.array([[1.0, 0.1], [-0.06, 0.99]]),
                    np.array([0.0, 0.02]),
                    np.zeros(2),
                )
                mpc.setStateWeights(np.array([1.0, scale]))
            # prepare_time is reported by the solve that follows prepare()
            before = prepared.getSolveInfo().prepare_time
            assert prepared.prepare() == ampc.SolveStatus.Success
            assert prepared.getSolveInfo().prepare_time == before
            assert prepared.solve(x0) == ampc.SolveStatus.Success
            assert prepared.getSolveInfo().prepare_time > 0.0
            assert direct.solve(x0) == ampc.SolveStatus.Success
            np.testing.assert_array_equal(
                prepared.getInputControlPoints(), direct.getInputControlPoints()
            )
            assert direct.getSolveInfo().prepare_time == 0.0


def test_prepare_time_covers_one_cycle():
    import numpy as np

    mpc = _configured_mpc(ampc.Formulation.Condensed)
    assert mpc.initializeSolver()
    x0 = np.array([0.3, -0.1])
    assert mpc.solve(x0) == ampc.SolveStatus.Success
    assert mpc.getSolveInfo().prepare_time == 0.0

    mpc.setStateWeights(np.array([1.0, 2.0]))
    assert mpc.prepare() == ampc.SolveStatus.Success
    assert mpc.solve(x0) == ampc.SolveStatus.Success
    assert mpc.getSolveInfo().prepare_time > 0.0

    # the next solve without prepare() reports zero again
    assert mpc.solve(x0) == ampc.SolveStatus.Success
    assert mpc.getSolveInfo().prepare_time == 0.0
