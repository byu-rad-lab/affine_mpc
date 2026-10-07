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

    mpc = ampc.CondensedMPC(2, 1, 10)
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
    assert info.run_time >= info.solve_time >= 0.0
    assert repr(info).startswith("SolveInfo(status=Success, iterations=")
