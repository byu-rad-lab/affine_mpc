#include "affine_mpc_py_module.hpp"

#include <pybind11/native_enum.h>
#include <pybind11/pybind11.h>
#include <sstream>

#include "affine_mpc/solve_status.hpp"

namespace affine_mpc_py {
namespace ampc = affine_mpc;
namespace py = pybind11;

void moduleAddSolveStatus(py::module& m)
{
  py::native_enum<ampc::SolveStatus>(
      m, "SolveStatus", "enum.Enum",
      "Enum for reporting the status of an MPC solve.")
      .value("Success", ampc::SolveStatus::Success, "MPC solved successfully")
      .value("NotInitialized", ampc::SolveStatus::NotInitialized,
             "MPC solver not initialized - must call initializeSolver() first")
      .value("SolvedInaccurate", ampc::SolveStatus::SolvedInaccurate,
             "The solver stopped early with a solution that meets only "
             "relaxed tolerances. For OSQP: it hit max_iter or the time "
             "limit, with residuals within 10 times eps_abs and eps_rel "
             "(see SolveInfo's residuals).")
      .value("PrimalInfeasible", ampc::SolveStatus::PrimalInfeasible,
             "OSQP reported value - see their docs")
      .value("DualInfeasible", ampc::SolveStatus::DualInfeasible,
             "OSQP reported value - see their docs")
      .value("MaxIterReached", ampc::SolveStatus::MaxIterReached,
             "OSQP reported value - see their docs")
      .value("TimeLimitReached", ampc::SolveStatus::TimeLimitReached,
             "OSQP reported value - see their docs")
      .value("OtherFailure", ampc::SolveStatus::OtherFailure,
             "OSQP reported value - see their docs")
      .value("UpdateFailed", ampc::SolveStatus::UpdateFailed,
             "OSQP rejected a QP data update, so the QP was not solved (often "
             "from non-finite model or weight values)")
      .finalize();

  py::class_<ampc::SolveInfo>(m, "SolveInfo",
                              R"doc(
Solver diagnostics from the most recent solve. Obtain with
MPC.getSolveInfo(). mpc_solve_time - qp_solve_time is the time spent
outside the QP solver (QP assembly, data updates, and refactorization). Work
done in MPC.prepare() is reported separately in prepare_time.
)doc")
      .def_readonly("status", &ampc::SolveInfo::status,
                    "Result of the solve. NotInitialized before "
                    "initializeSolver(), and OtherFailure after "
                    "initialization but before the first solve.")
      .def_readonly("iterations", &ampc::SolveInfo::iterations,
                    "Number of solver iterations.")
      .def_readonly("objective", &ampc::SolveInfo::objective,
                    "QP objective value 1/2 z^T P z + q^T z at the solution. "
                    "Omits constant terms of the MPC cost, so it is not the "
                    "full MPC cost.")
      .def_readonly("qp_solve_time", &ampc::SolveInfo::qp_solve_time,
                    "Time the QP solver spent solving, including polishing "
                    "(seconds). Zero when OSQP is built without profiling.")
      .def_readonly("mpc_solve_time", &ampc::SolveInfo::mpc_solve_time,
                    "Duration of the whole last solve() call after "
                    "initialization, including QP assembly, data updates, "
                    "refactorization, and the solver (seconds). Does not "
                    "depend on OSQP profiling.")
      .def_readonly("prepare_time", &ampc::SolveInfo::prepare_time,
                    "Total duration of the prepare() calls between the "
                    "previous solve() and this one (seconds), zero if there "
                    "were none. prepare_time + mpc_solve_time is the MPC's "
                    "work for the step; mpc_solve_time alone is its latency.")
      .def_readonly("primal_residual", &ampc::SolveInfo::primal_residual,
                    "Primal residual of the solution: the largest violation "
                    "of l <= A z <= u (max norm), as the solver reports it. "
                    "NaN if the solver does not report it, or before the "
                    "first solve.")
      .def_readonly("dual_residual", &ampc::SolveInfo::dual_residual,
                    "Dual residual of the solution: max norm of "
                    "P z + q + A^T y, with y from MPC.getDualSolution(), as "
                    "the solver reports it. NaN if the solver does not report "
                    "it, or before the first solve. Both residuals are "
                    "absolute: compare them with your own tolerance rather "
                    "than reading them as pass or fail.")
      .def("__repr__", [](const ampc::SolveInfo& self) {
        std::ostringstream oss;
        oss << self;
        return oss.str();
      });
}

} // namespace affine_mpc_py
