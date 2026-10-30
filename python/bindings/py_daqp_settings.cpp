#include "affine_mpc_py_module.hpp"

#include <pybind11/pybind11.h>
#include <sstream>

#include "affine_mpc/daqp_settings.hpp"

namespace affine_mpc_py {
namespace ampc = affine_mpc;
namespace py = pybind11;

void moduleAddDaqpSettings(py::module& m)
{
  py::class_<ampc::DAQPSettings> s(m, "DAQPSettings", R"doc(
DAQP solver settings; the defaults are DAQP's.

Pass them to MPC.initializeSolver() to solve with DAQP, a dense active-set
solver, instead of OSQP (condensed formulation only). The class exists in every
build, but initializing with it needs a library built with DAQP; check with
hasDAQP().

Attributes:
    primal_tol: Tolerance for primal infeasibility: constraints may be violated
        by up to this much.
    dual_tol: Tolerance for dual infeasibility (the sign of the multipliers).
    iter_limit: Maximum number of active-set iterations.
    time_limit: Time limit for a solve (seconds); zero or negative for none.
    eps_prox: Proximal-point regularization, which lets DAQP solve QPs whose
        cost matrix P is only positive semidefinite (singular). Negative (the
        default, -1e-6): automatic, used only when P is singular or too
        ill-conditioned to factorize, starting from |eps_prox|. Positive: always
        used, with this eps. Zero: never used, so a singular P makes
        initializeSolver() return False (or, after a weight or model change,
        the next solve return UpdateFailed).
)doc");

  s.def(py::init<>(), "Constructor sets DAQP's default values.");
  s.def_readwrite("primal_tol", &ampc::DAQPSettings::primal_tol);
  s.def_readwrite("dual_tol", &ampc::DAQPSettings::dual_tol);
  s.def_readwrite("iter_limit", &ampc::DAQPSettings::iter_limit);
  s.def_readwrite("time_limit", &ampc::DAQPSettings::time_limit);
  s.def_readwrite("eps_prox", &ampc::DAQPSettings::eps_prox);
  s.def("__repr__", [](const ampc::DAQPSettings& self) {
    std::ostringstream oss;
    oss << "DAQPSettings(primal_tol=" << self.primal_tol
        << ", dual_tol=" << self.dual_tol << ", iter_limit=" << self.iter_limit
        << ", time_limit=" << self.time_limit << ", eps_prox=" << self.eps_prox
        << ')';
    return oss.str();
  });

  m.def("hasDAQP", &ampc::hasDAQP, R"doc(
Whether the library was built with DAQP, so that MPC.initializeSolver() accepts
DAQPSettings. The wheels on PyPI are built with it.
)doc");
}

} // namespace affine_mpc_py
