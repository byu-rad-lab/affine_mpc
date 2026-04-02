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
    except:
        assert False


if __name__ == "__main__":
    test_solve_status_interface()
