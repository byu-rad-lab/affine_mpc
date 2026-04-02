import affine_mpc as ampc


def test_osqp_settings_interface():
    try:
        settings = ampc.OSQPSettings()
        # Default values
        settings.rho = 0.1
        settings.sigma = 1e-6
        settings.scaling = 10

        settings.adaptive_rho = 1
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
        settings.polishing = 0
        settings.polish_refine_iter = 3
        settings.verbose = 1

        settings.scaled_termination = 0
        settings.check_termination = 25
        settings.warm_starting = 1

        settings.time_limit = 0  # disabled
    except:
        assert False


if __name__ == "__main__":
    test_osqp_settings_interface()
    print("All tests passed!")
