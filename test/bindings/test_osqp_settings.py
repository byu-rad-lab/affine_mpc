import affine_mpc as ampc


def test_osqp_settings_interface():
    try:
        settings = ampc.OSQPSettings()
        # Default values
        settings.rho = 0.1
        settings.sigma = 1e-6
        settings.scaling = 10

        settings.adaptive_rho = settings.AdaptiveRhoType.Iterations
        settings.adaptive_rho_interval = 0
        settings.adaptive_rho_tolerance = 5
        settings.adaptive_rho_fraction = 0.4

        settings.max_iter = 4000
        settings.eps_abs = 1e-3
        settings.eps_rel = 1e-3
        settings.eps_prim_inf = 1e-4
        settings.eps_dual_inf = 1e-4
        settings.alpha = 1.6
        settings.linsys_solver = ampc.OSQPSettings.LinsysSolverType.DirectSolver
        settings.linsys_solver = settings.LinsysSolverType.IndirectSolver
        settings.linsys_solver = settings.LinsysSolverType.UnknownSolver

        settings.delta = 1e-6
        settings.polishing = False
        settings.polish_refine_iter = 3
        settings.verbose = True

        settings.scaled_termination = False
        settings.check_termination = 25
        settings.warm_starting = True

        settings.time_limit = 0  # disabled
    except:
        assert False


def test_osqp_settings_defaults():
    defaults = ampc.OSQPSettings.fromOSQPDefaults()
    assert defaults.verbose is True
    assert defaults.warm_starting is True
    assert defaults.polishing is False
    assert defaults.scaling == 10
    assert defaults.max_iter == 4000
    assert defaults.eps_abs == 1e-3
    assert defaults.eps_rel == 1e-3
    assert defaults.rho == 0.1
    assert defaults.time_limit == 1e10
    assert defaults.linsys_solver == ampc.OSQPSettings.LinsysSolverType.DirectSolver
    assert (
        defaults.cg_precond
        == ampc.OSQPSettings.PreconditionerType.DiagonalPreconditioner
    )

    recommended = ampc.OSQPSettings()
    assert recommended.verbose is False
    assert recommended.eps_abs == 1e-6
    assert recommended.eps_rel == 1e-6
    assert recommended.eps_dual_inf == 1e-6
    assert recommended.eps_prim_inf == defaults.eps_prim_inf
    assert recommended.max_iter == defaults.max_iter


def test_osqp_settings_repr_uses_recommended_defaults():
    settings = ampc.OSQPSettings()
    assert repr(settings) == "OSQPSettings()"

    settings.verbose = True
    settings.eps_abs = 1e-4
    assert repr(settings) == "OSQPSettings(eps_abs=0.0001, verbose=True)"

    settings = ampc.OSQPSettings()
    settings.adaptive_rho = ampc.OSQPSettings.AdaptiveRhoType.KKTError
    assert (
        repr(settings)
        == "OSQPSettings(adaptive_rho=OSQPSettings.AdaptiveRhoType.KKTError)"
    )
    assert "adaptive_rho = KKTError\n" in str(settings)


if __name__ == "__main__":
    test_osqp_settings_interface()
    print("All tests passed!")
