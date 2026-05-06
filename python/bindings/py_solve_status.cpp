#include "affine_mpc_py_module.hpp"

#include <pybind11/native_enum.h>
#include <pybind11/pybind11.h>

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
}

} // namespace affine_mpc_py
