#include "affine_mpc_py_module.hpp"

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

#include "affine_mpc/parameterization.hpp"
#include "affine_mpc/sparse_mpc.hpp"

namespace affine_mpc_py {
namespace ampc = affine_mpc;
namespace py = pybind11;


void moduleAddSparseMPC(py::module& m)
{
  py::class_<ampc::SparseMPC, ampc::MPCBase> mpc(m, "SparseMPC",
                                                 R"doc(
MPC formulation where the predicted state trajectory and parameterization
control points are both optimization design variables (sparse QP).

Retains state and input variables with model as an optimization constraint,
exploiting sparsity for scalability. Preferred for longer horizons or larger
state dimensions. Converts the MPC problem to QP form for OSQP, using input
parameterization.
                                                 )doc");

  mpc.def(py::init<const int, const int, const ampc::Parameterization&,
                   const ampc::Options&>(),
          R"doc(
Construct SparseMPC with specified input parameterization and MPC
configuration options.

Args:
   state_dim: State vector dimension.
   input_dim: Input vector dimension.
   param: Input trajectory parameterization.
   opts: Optional MPC configuration features to enable.
           )doc",
          py::arg("state_dim"), py::arg("input_dim"), py::arg("param"),
          py::arg("opts") = ampc::Options());

  mpc.def(py::init<const int, const int, const int, const ampc::Options&>(),
          R"doc(
Construct SparseMPC with no parameterization (full input trajectory will be
optimized) and MPC configuration options.

Args:
   state_dim: State vector dimension.
   input_dim: Input vector dimension.
   horizon_steps: Number of discrete steps in the MPC horizon.
   opts: Optional MPC configuration features to enable.
           )doc",
          py::arg("state_dim"), py::arg("input_dim"), py::arg("horizon_steps"),
          py::arg("opts") = ampc::Options());
}

} // namespace affine_mpc_py
