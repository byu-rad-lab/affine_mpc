#include "affine_mpc_py_module.hpp"

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

#include "affine_mpc/condensed_mpc.hpp"
#include "affine_mpc/options.hpp"
#include "affine_mpc/parameterization.hpp"

namespace affine_mpc_py {
namespace ampc = affine_mpc;
namespace py = pybind11;


void moduleAddCondensedMPC(py::module& m)
{
  py::class_<ampc::CondensedMPC, ampc::MPCBase> mpc(m, "CondensedMPC",
                                                    R"doc(
MPC formulation where only parameterization control points are optimization
design variables (condensed QP).

Eliminates state variables analytically by implicitly wrapping the model into
the const function rather than as a constraint , resulting in a smaller dense
QP. Preferred for shorter horizons and lower-dimensional problems.
Converts the MPC problem to QP form for OSQP, using input parameterization.
                                         )doc");

  mpc.def(py::init<const int, const int, const ampc::Parameterization&,
                   const ampc::Options&>(),
          R"doc(
Construct CondensedMPC with specified input parameterization and MPC
configuration options.

Args:
   state_dim: State vector dimension.
   input_dim: Input vector dimension.
   param: Input trajectory parameterization.
   opts: Optional MPC configuration features to enable.
           )doc",
          py::arg("state_dim"), py::arg("input_dim"), py::arg("param"),
          py::arg("opts") = ampc::Options{});

  mpc.def(py::init<const int, const int, const int, const ampc::Options&>(),
          R"doc(
Construct CondensedMPC with no parameterization (full input trajectory will be
optimized) and MPC configuration options.

Args:
   state_dim: State vector dimension.
   input_dim: Input vector dimension.
   horizon_steps: Number of discrete steps in the MPC horizon.
   opts: Optional MPC configuration features to enable.
           )doc",
          py::arg("state_dim"), py::arg("input_dim"), py::arg("horizon_steps"),
          py::arg("opts") = ampc::Options{});
}

} // namespace affine_mpc_py
