#include "affine_mpc_py_module.hpp"

#include <osqp.h>
#include <pybind11/pybind11.h>

#include "affine_mpc/osqp_solver.hpp"
#include "osqp_api_constants.h"

namespace affine_mpc_py {
namespace ampc = affine_mpc;
namespace py = pybind11;

void moduleAddOsqpSettings(py::module& m)
{
  py::class_<OSQPSettings> s(m, "OSQPSettings", R"doc(
OSQP solver settings.

Most users will likely not need to modify the default settings, but they are
exposed here for advanced use cases. See OSQP documentation for details on each
setting.

Note: `eps_abs` and `eps_rel` are the main tolerances for convergence.
Tightening these can improve solution accuracy but may increase solve time.
Defaults are usually sufficient for MPC applications, but can be adjusted if
convergence is an issue. OSQP default is 1e-3 while affine_mpc defaults to 1e-6.

Attributes:
    adaptive_rho: Enable adaptive ADMM step size.
    adaptive_rho_fraction: Fraction of primal and dual residuals to trigger rho update.
    adaptive_rho_interval: Minimum number of iterations between rho updates.
    adaptive_rho_tolerance: Absolute residual tolerance for adaptive rho updates.
    alpha: Relaxation parameter. Default for QP solvers is 1.6.
        =1: no relaxation (standard ADMM).
        >1: over-relaxation (can improve convergence in some cases).
        <1: under-relaxation (rarely useful).

        If iters is high, consider increasing to 1.6-1.8. If oscillations occur,
        consider decreasing to 1.4-1.5. Can also try 1.0 for standard ADMM if
        convergence is an issue.
    check_termination: Check termination conditions at each iteration.
    delta: Regularization parameter for linear system solver.
    eps_abs: Absolute tolerance for termination.
    eps_dual_inf: Dual infeasibility tolerance for termination.
    eps_prim_inf: Primal infeasibility tolerance for termination.
    eps_rel: Relative tolerance for termination.
    linsys_solver: Linear system solver to use (direct, indirect, or unknown).
    max_iter: Maximum number of iterations.
    polishing: Enable polishing of the solution, which performs an additional
        solve after convergence to polish the solution. This can improve
        accuracy near,constraint limits, but adds computation time. For example,
        if the solution is at input saturation, tolerances may cause the
        solution to be slightly outside the limits, and polishing can help
        ensure the final solution respects the limits more accurately.
        An alternative to polishing is to saturate the solution after solving.
    polish_refine_iter: Number of refinement iterations for polishing.
    rho: ADMM step size.
    scaled_termination: Check termination conditions on scaled residuals.
    scaling: Enable problem data scaling.
    sigma: ADMM regularization parameter.
    time_limit: Time limit for solver in seconds.
    verbose: Enable verbose output from OSQP for each solve, which may be useful
        for debugging. For general MPC usage, this should be false.
    warm_starting: Enable warm starting with previous solution.
                             )doc");

  // must define LinsysSolverType before using below
  py::enum_<osqp_linsys_solver_type>(s, "LinsysSolverType")
      .value("DirectSolver", osqp_linsys_solver_type::OSQP_DIRECT_SOLVER)
      .value("IndirectSolver", osqp_linsys_solver_type::OSQP_INDIRECT_SOLVER)
      .value("UnknownSolver", osqp_linsys_solver_type::OSQP_UNKNOWN_SOLVER);

  s.def_static(
      "fromOSQPDefaults", &ampc::OSQPSolver::getDefaultSettings,
      "Factory method to create OSQPSettings with OSQP default values.");

  s.def(py::init<>([]() { return ampc::OSQPSolver::getRecommendedSettings(); }),
        "Constructor sets affine_mpc recommended values.");
  s.def_readwrite("adaptive_rho", &OSQPSettings::adaptive_rho);
  s.def_readwrite("adaptive_rho_fraction",
                  &OSQPSettings::adaptive_rho_fraction);
  s.def_readwrite("adaptive_rho_interval",
                  &OSQPSettings::adaptive_rho_interval);
  s.def_readwrite("adaptive_rho_tolerance",
                  &OSQPSettings::adaptive_rho_tolerance);
  s.def_readwrite("alpha", &OSQPSettings::alpha);
  s.def_readwrite("check_termination", &OSQPSettings::check_termination);
  s.def_readwrite("delta", &OSQPSettings::delta);
  s.def_readwrite("eps_abs", &OSQPSettings::eps_abs);
  s.def_readwrite("eps_dual_inf", &OSQPSettings::eps_dual_inf);
  s.def_readwrite("eps_prim_inf", &OSQPSettings::eps_prim_inf);
  s.def_readwrite("eps_rel", &OSQPSettings::eps_rel);
  // Works but user can't pass in an int for the solver type enum
  s.def_readwrite("linsys_solver", &OSQPSettings::linsys_solver);
  // Allows user to pass in an int for the solver type enum
  // s.def_property(
  //     "linsys_solver",
  //     [](const OSQPSettings& self) { return self.linsys_solver; },
  //     [](OSQPSettings& self, osqp_linsys_solver_type type_linsys_solver) {
  //       switch (type_linsys_solver) {
  //       case osqp_linsys_solver_type::OSQP_DIRECT_SOLVER:
  //       case osqp_linsys_solver_type::OSQP_INDIRECT_SOLVER:
  //       case osqp_linsys_solver_type::OSQP_UNKNOWN_SOLVER:
  //         self.linsys_solver =
  //             static_cast<osqp_linsys_solver_type>(type_linsys_solver);
  //         break;
  //       default:
  //         throw std::invalid_argument("Invalid LinsysSolverType");
  //       }
  //     });
  s.def_readwrite("max_iter", &OSQPSettings::max_iter);
  s.def_readwrite("polishing", &OSQPSettings::polishing);
  s.def_readwrite("polish_refine_iter", &OSQPSettings::polish_refine_iter);
  s.def_readwrite("rho", &OSQPSettings::rho);
  s.def_readwrite("scaled_termination", &OSQPSettings::scaled_termination);
  s.def_readwrite("scaling", &OSQPSettings::scaling);
  s.def_readwrite("sigma", &OSQPSettings::sigma);
  s.def_readwrite("time_limit", &OSQPSettings::time_limit);
  s.def_readwrite("verbose", &OSQPSettings::verbose);
  s.def_readwrite("warm_starting", &OSQPSettings::warm_starting);
}

} // namespace affine_mpc_py
