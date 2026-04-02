#include "affine_mpc_py_module.hpp"

#include <pybind11/pybind11.h>

#include "affine_mpc/solve_status.hpp"

namespace affine_mpc_py {
namespace ampc = affine_mpc;
namespace py = pybind11;

void moduleAddSolveStatus(py::module& m)
{
  py::enum_<ampc::SolveStatus> status(m, "SolveStatus");

  status.value("Success", ampc::SolveStatus::Success);
  status.value("NotInitialized", ampc::SolveStatus::NotInitialized);
  status.value("SolvedInaccurate", ampc::SolveStatus::SolvedInaccurate);
  status.value("PrimalInfeasible", ampc::SolveStatus::PrimalInfeasible);
  status.value("DualInfeasible", ampc::SolveStatus::DualInfeasible);
  status.value("MaxIterReached", ampc::SolveStatus::MaxIterReached);
  status.value("TimeLimitReached", ampc::SolveStatus::TimeLimitReached);
  status.value("OtherFailure", ampc::SolveStatus::OtherFailure);
}

} // namespace affine_mpc_py
