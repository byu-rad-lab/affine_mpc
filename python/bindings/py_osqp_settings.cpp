#include "affine_mpc_py_module.hpp"

#include <pybind11/native_enum.h>
#include <pybind11/pybind11.h>
#include <sstream>

#include "affine_mpc/osqp_settings.hpp"
#include "affine_mpc/osqp_solver.hpp"

namespace affine_mpc_py {
namespace ampc = affine_mpc;
namespace py = pybind11;
using Linsys = ampc::OSQPSettings::LinsysSolverType;
using Precond = ampc::OSQPSettings::PreconditionerType;
using AdaptiveRho = ampc::OSQPSettings::AdaptiveRhoType;

namespace {
const char* adaptiveRhoName(const AdaptiveRho type)
{
  switch (type) {
  case AdaptiveRho::Disabled:
    return "Disabled";
  case AdaptiveRho::Time:
    return "Time";
  case AdaptiveRho::KKTError:
    return "KKTError";
  default:
    return "Iterations";
  }
}
} // namespace

void moduleAddOsqpSettings(py::module& m)
{
  py::class_<ampc::OSQPSettings> s(m, "OSQPSettings", R"doc(
OSQP solver settings.

Most users will likely not need to modify the default settings, but they are
exposed here for advanced use cases. See OSQP documentation for details on each
setting: https://osqp.org/docs/interfaces/solver_settings.html.

Note: `eps_abs` and `eps_rel` are the main tolerances for convergence.
Tightening these can improve solution accuracy but may increase solve time.
Defaults are usually sufficient for MPC applications, but can be adjusted if
convergence is an issue. OSQP default is 1e-3 while affine_mpc defaults to 1e-6.
affine_mpc also tightens `eps_dual_inf` to 1e-6 (OSQP: 1e-4), since a looser
value can report a badly conditioned but solvable QP as DualInfeasible.

Attributes:
    adaptive_rho: When rho is adapted (AdaptiveRhoType): Disabled, every
        adaptive_rho_interval iterations (Iterations), after a fraction of
        the setup time (Time, needs OSQP built with profiling), or when the
        KKT error has decreased to adaptive_rho_fraction of its value at the
        last adaptation (KKTError).
    adaptive_rho_fraction: Fraction of the setup time (Time) or of the KKT
        error at the last adaptation (KKTError) at which rho is adapted.
    adaptive_rho_interval: Iterations between rho adaptations (Iterations);
        zero chooses it from check_termination.
    adaptive_rho_tolerance: Minimum ratio between a new rho and the current
        one to change it.
    alpha: Relaxation parameter. Default for QP solvers is 1.6.
        =1: no relaxation (standard ADMM).
        >1: over-relaxation (can improve convergence in some cases).
        <1: under-relaxation (rarely useful).

        If iters is high, consider increasing to 1.6-1.8. If oscillations occur,
        consider decreasing to 1.4-1.5. Can also try 1.0 for standard ADMM if
        convergence is an issue.
    cg_max_iter: Maximum iterations for conjugate gradient linear system solver.
    cg_precond: Preconditioner type for conjugate gradient solver.
    cg_tol_fraction: Conjugate gradient tolerance as fraction of initial residual.
    cg_tol_reduction: Number of conjugate gradient iterations before tol is halved.
    check_dualgap: Check duality gap at each iteration for termination.
    check_termination: Check termination interval.
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
    profiler_level: Level of detail for profiler annotations.
    rho: ADMM step size.
    rho_is_vec: Whether rho is a scalar or vector.
    scaled_termination: Check termination conditions on scaled residuals.
    scaling: Enable problem data scaling.
    sigma: ADMM regularization parameter.
    time_limit: Time limit for solver in seconds.
    verbose: Enable verbose output from OSQP for each solve, which may be useful
        for debugging. For general MPC usage, this should be false.
    warm_starting: Enable warm starting with previous solution.
                             )doc");

  // must define LinsysSolverType before using below
  py::native_enum<ampc::OSQPSettings::LinsysSolverType>(
      s, "LinsysSolverType", "enum.Enum", "Enum for linear system solver type.")
      .value("DirectSolver", Linsys::DirectSolver)
      .value("IndirectSolver", Linsys::IndirectSolver)
      .value("UnknownSolver", Linsys::UnknownSolver)
      .finalize();

  py::native_enum<Precond>(s, "PreconditionerType", "enum.Enum",
                           "Enum for preconditioner type.")
      .value("NoPreconditioner", Precond::NoPreconditioner)
      .value("DiagonalPreconditioner", Precond::DiagonalPreconditioner)
      .finalize();

  py::native_enum<AdaptiveRho>(s, "AdaptiveRhoType", "enum.Enum",
                               "When OSQP adapts rho.")
      .value("Disabled", AdaptiveRho::Disabled)
      .value("Iterations", AdaptiveRho::Iterations)
      .value("Time", AdaptiveRho::Time)
      .value("KKTError", AdaptiveRho::KKTError)
      .finalize();

  s.def_static(
      "fromOSQPDefaults", &ampc::OSQPSolver::getDefaultSettings,
      "Factory method to create OSQPSettings with OSQP default values.");

  s.def(py::init<>([]() { return ampc::OSQPSolver::getRecommendedSettings(); }),
        "Constructor sets affine_mpc recommended values.");
  s.def_readwrite("adaptive_rho", &ampc::OSQPSettings::adaptive_rho);
  s.def_readwrite("adaptive_rho_fraction",
                  &ampc::OSQPSettings::adaptive_rho_fraction);
  s.def_readwrite("adaptive_rho_interval",
                  &ampc::OSQPSettings::adaptive_rho_interval);
  s.def_readwrite("adaptive_rho_tolerance",
                  &ampc::OSQPSettings::adaptive_rho_tolerance);
  s.def_readwrite("alpha", &ampc::OSQPSettings::alpha);
  s.def_readwrite("cg_max_iter", &ampc::OSQPSettings::cg_max_iter);
  s.def_readwrite("cg_precond", &ampc::OSQPSettings::cg_precond);
  s.def_readwrite("cg_tol_fraction", &ampc::OSQPSettings::cg_tol_fraction);
  s.def_readwrite("cg_tol_reduction", &ampc::OSQPSettings::cg_tol_reduction);
  s.def_readwrite("check_dualgap", &ampc::OSQPSettings::check_dualgap);
  s.def_readwrite("check_termination", &ampc::OSQPSettings::check_termination);
  s.def_readwrite("delta", &ampc::OSQPSettings::delta);
  s.def_readwrite("eps_abs", &ampc::OSQPSettings::eps_abs);
  s.def_readwrite("eps_dual_inf", &ampc::OSQPSettings::eps_dual_inf);
  s.def_readwrite("eps_prim_inf", &ampc::OSQPSettings::eps_prim_inf);
  s.def_readwrite("eps_rel", &ampc::OSQPSettings::eps_rel);
  s.def_readwrite("linsys_solver", &ampc::OSQPSettings::linsys_solver);
  s.def_readwrite("max_iter", &ampc::OSQPSettings::max_iter);
  s.def_readwrite("polishing", &ampc::OSQPSettings::polishing);
  s.def_readwrite("polish_refine_iter",
                  &ampc::OSQPSettings::polish_refine_iter);
  s.def_readwrite("profiler_level", &ampc::OSQPSettings::profiler_level);
  s.def_readwrite("rho", &ampc::OSQPSettings::rho);
  s.def_readwrite("rho_is_vec", &ampc::OSQPSettings::rho_is_vec);
  s.def_readwrite("scaled_termination",
                  &ampc::OSQPSettings::scaled_termination);
  s.def_readwrite("scaling", &ampc::OSQPSettings::scaling);
  s.def_readwrite("sigma", &ampc::OSQPSettings::sigma);
  s.def_readwrite("time_limit", &ampc::OSQPSettings::time_limit);
  s.def_readwrite("verbose", &ampc::OSQPSettings::verbose);
  s.def_readwrite("warm_starting", &ampc::OSQPSettings::warm_starting);

  s.def("__str__", [](const ampc::OSQPSettings& self) {
    std::ostringstream os;
    const auto tf = [](bool b) { return b ? "True" : "False"; };
    auto linsysSolverStr = [](const Linsys solver) {
      switch (solver) {
      case Linsys::DirectSolver:
        return "DirectSolver";
      case Linsys::IndirectSolver:
        return "IndirectSolver";
      default:
        return "UnknownSolver";
      }
    };
    auto precondStr = [](const Precond precond) {
      switch (precond) {
      case Precond::NoPreconditioner:
        return "NoPreconditioner";
      default:
        return "DiagonalPreconditioner";
      }
    };

    os << "OSQP Solver Settings:\n"
       << "  adaptive_rho = " << adaptiveRhoName(self.adaptive_rho) << "\n"
       << "  adaptive_rho_fraction = " << self.adaptive_rho_fraction << "\n"
       << "  adaptive_rho_interval = " << self.adaptive_rho_interval << "\n"
       << "  adaptive_rho_tolerance = " << self.adaptive_rho_tolerance << "\n"
       << "  alpha = " << self.alpha << "\n"
       << "  cg_max_iter = " << self.cg_max_iter << "\n"
       << "  cg_precond = " << precondStr(self.cg_precond) << "\n"
       << "  cg_tol_fraction = " << self.cg_tol_fraction << "\n"
       << "  cg_tol_reduction = " << self.cg_tol_reduction << "\n"
       << "  check_dualgap = " << tf(self.check_dualgap) << "\n"
       << "  check_termination = " << self.check_termination << "\n"
       << "  delta = " << self.delta << "\n"
       << "  device = " << self.device << "\n"
       << "  eps_abs = " << self.eps_abs << "\n"
       << "  eps_dual_inf = " << self.eps_dual_inf << "\n"
       << "  eps_prim_inf = " << self.eps_prim_inf << "\n"
       << "  eps_rel = " << self.eps_rel << "\n"
       << "  linsys_solver = " << linsysSolverStr(self.linsys_solver) << "\n"
       << "  max_iter = " << self.max_iter << "\n"
       << "  polishing = " << tf(self.polishing) << "\n"
       << "  polish_refine_iter = " << self.polish_refine_iter << "\n"
       << "  profiler_level = " << self.profiler_level << "\n"
       << "  rho = " << self.rho << "\n"
       << "  rho_is_vec = " << tf(self.rho_is_vec) << "\n"
       << "  scaled_termination = " << tf(self.scaled_termination) << "\n"
       << "  scaling = " << self.scaling << "\n"
       << "  sigma = " << self.sigma << "\n"
       << "  time_limit = " << self.time_limit << "\n"
       << "  verbose = " << tf(self.verbose) << "\n"
       << "  warm_starting = " << tf(self.warm_starting);
    return os.str();
  });

  s.def("__repr__", [](const ampc::OSQPSettings& self) {
    const ampc::OSQPSettings defaults{
        ampc::OSQPSolver::getRecommendedSettings()};
    std::ostringstream oss;

    oss << "OSQPSettings(";
    bool prev{false};
    auto addSeparator = [&]() {
      if (prev) {
        oss << ", ";
      }
      prev = true;
    };
    auto addBool = [&](const char* name, const bool value) {
      addSeparator();
      oss << name << '=' << (value ? "True" : "False");
    };
    auto addValue = [&](const char* name, const auto& value) {
      addSeparator();
      oss << name << '=' << value;
    };
    auto addLinsysSolver = [&](const Linsys value) {
      addSeparator();
      oss << "linsys_solver=OSQPSettings.LinsysSolverType.";
      switch (value) {
      case Linsys::DirectSolver:
        oss << "DirectSolver";
        break;
      case Linsys::IndirectSolver:
        oss << "IndirectSolver";
        break;
      default:
        oss << "UnknownSolver";
        break;
      }
    };
    auto addPreconditioner = [&](const Precond value) {
      addSeparator();
      oss << "cg_precond=OSQPSettings.PreconditionerType.";
      switch (value) {
      case Precond::NoPreconditioner:
        oss << "NoPreconditioner";
        break;
      default:
        oss << "DiagonalPreconditioner";
        break;
      }
    };

    if (self.adaptive_rho != defaults.adaptive_rho) {
      addSeparator();
      oss << "adaptive_rho=OSQPSettings.AdaptiveRhoType."
          << adaptiveRhoName(self.adaptive_rho);
    }
    if (self.adaptive_rho_fraction != defaults.adaptive_rho_fraction)
      addValue("adaptive_rho_fraction", self.adaptive_rho_fraction);
    if (self.adaptive_rho_interval != defaults.adaptive_rho_interval)
      addValue("adaptive_rho_interval", self.adaptive_rho_interval);
    if (self.adaptive_rho_tolerance != defaults.adaptive_rho_tolerance)
      addValue("adaptive_rho_tolerance", self.adaptive_rho_tolerance);
    if (self.alpha != defaults.alpha)
      addValue("alpha", self.alpha);
    if (self.cg_max_iter != defaults.cg_max_iter)
      addValue("cg_max_iter", self.cg_max_iter);
    if (self.cg_precond != defaults.cg_precond)
      addPreconditioner(self.cg_precond);
    if (self.cg_tol_fraction != defaults.cg_tol_fraction)
      addValue("cg_tol_fraction", self.cg_tol_fraction);
    if (self.cg_tol_reduction != defaults.cg_tol_reduction)
      addValue("cg_tol_reduction", self.cg_tol_reduction);
    if (self.check_dualgap != defaults.check_dualgap)
      addBool("check_dualgap", self.check_dualgap);
    if (self.check_termination != defaults.check_termination)
      addValue("check_termination", self.check_termination);
    if (self.delta != defaults.delta)
      addValue("delta", self.delta);
    if (self.eps_abs != defaults.eps_abs)
      addValue("eps_abs", self.eps_abs);
    if (self.eps_dual_inf != defaults.eps_dual_inf)
      addValue("eps_dual_inf", self.eps_dual_inf);
    if (self.eps_prim_inf != defaults.eps_prim_inf)
      addValue("eps_prim_inf", self.eps_prim_inf);
    if (self.eps_rel != defaults.eps_rel)
      addValue("eps_rel", self.eps_rel);
    if (self.linsys_solver != defaults.linsys_solver)
      addLinsysSolver(self.linsys_solver);
    if (self.max_iter != defaults.max_iter)
      addValue("max_iter", self.max_iter);
    if (self.polishing != defaults.polishing)
      addBool("polishing", self.polishing);
    if (self.polish_refine_iter != defaults.polish_refine_iter)
      addValue("polish_refine_iter", self.polish_refine_iter);
    if (self.profiler_level != defaults.profiler_level)
      addValue("profiler_level", self.profiler_level);
    if (self.rho != defaults.rho)
      addValue("rho", self.rho);
    if (self.rho_is_vec != defaults.rho_is_vec)
      addBool("rho_is_vec", self.rho_is_vec);
    if (self.scaled_termination != defaults.scaled_termination)
      addBool("scaled_termination", self.scaled_termination);
    if (self.scaling != defaults.scaling)
      addValue("scaling", self.scaling);
    if (self.sigma != defaults.sigma)
      addValue("sigma", self.sigma);
    if (self.time_limit != defaults.time_limit)
      addValue("time_limit", self.time_limit);
    if (self.verbose != defaults.verbose)
      addBool("verbose", self.verbose);
    if (self.warm_starting != defaults.warm_starting)
      addBool("warm_starting", self.warm_starting);
    oss << ')';
    return oss.str();
  });
}

} // namespace affine_mpc_py
