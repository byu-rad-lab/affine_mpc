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
             "OSQP reported value - see their docs")
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
      .finalize();

  py::class_<ampc::SolveInfo>(m, "SolveInfo",
                              R"doc(
Solver diagnostics from the most recent solve. Obtain with
MPCBase.getSolveInfo(). Timing fields are zero if OSQP was built without
profiling.
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
      .def_readonly("solve_time", &ampc::SolveInfo::solve_time,
                    "Time spent in the solve phase (seconds).")
      .def_readonly("run_time", &ampc::SolveInfo::run_time,
                    "Total solver time for the last solve, including data "
                    "updates and polishing (seconds).")
      .def("__repr__", [](const ampc::SolveInfo& self) {
        std::ostringstream oss;
        oss << self;
        return oss.str();
      });
}

} // namespace affine_mpc_py
