#include "affine_mpc_py_module.hpp"

#include <Eigen/Core>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <sstream>
#include <tuple>

#include "affine_mpc/discretizer.hpp"

namespace affine_mpc_py {
namespace ampc = affine_mpc;
namespace py = pybind11;

void moduleAddDiscretizer(py::module& m)
{
  py::class_<ampc::Discretizer> disc(m, "Discretizer", R"doc(
Discretizes a continuous-time affine model dx/dt = Ac x + Bc u + wc into
x_next = Ad x + Bd u + wd, assuming the input is held constant over each time
step (zero-order hold).

The result is exact up to the truncation tolerance: Ad = exp(Ac dt),
Bd = G Bc, and wd = G wc with G = integral_0^dt exp(Ac s) ds. The matrix
exponential is computed with scaling and squaring. The last G is kept, so a new
model offset can be discretized without recomputing the matrix exponential
(see discretizeOffset()). Useful on its own for simulation and for
updating the affine approximation of a nonlinear model.
)doc");

  disc.def(py::init<int, int>(), R"doc(
Construct a Discretizer for models of the given dimensions.

Args:
    state_dim: State vector dimension (positive).
    input_dim: Input vector dimension (positive).

Raises:
    ValueError: If a dimension is not positive.
)doc",
           py::arg("state_dim"), py::arg("input_dim"));

  disc.def(
      "discretize",
      [](ampc::Discretizer& self, const Eigen::Ref<const Eigen::MatrixXd>& Ac,
         const Eigen::Ref<const Eigen::MatrixXd>& Bc,
         const Eigen::Ref<const Eigen::VectorXd>& wc, double dt, double tol) {
        Eigen::MatrixXd Ad{self.getStateDim(), self.getStateDim()};
        Eigen::MatrixXd Bd{self.getStateDim(), self.getInputDim()};
        Eigen::VectorXd wd{self.getStateDim()};
        self.discretize(Ac, Bc, wc, dt, Ad, Bd, wd, tol);
        return std::make_tuple(Ad, Bd, wd);
      },
      R"doc(
Discretize a continuous-time affine model.

Args:
    Ac: Continuous-time state matrix (state_dim x state_dim).
    Bc: Continuous-time input matrix (state_dim x input_dim).
    wc: Continuous-time model offset (size state_dim).
    dt: Time step in seconds (positive).
    tol: Truncation tolerance for the Taylor series (positive). The actual
        error is typically two to three orders of magnitude smaller than tol.
        The default gives results close to machine precision; larger values
        (e.g., 1e-6) save a few matrix products.

Returns:
    (Ad, Bd, wd): Discrete-time state matrix, input matrix, and model offset.

Raises:
    ValueError: If a size is wrong, dt or tol is not positive, or Ac*dt is not
        finite.
)doc",
      py::arg("Ac"), py::arg("Bc"), py::arg("wc"), py::arg("dt"),
      py::arg("tol") = 1e-10);

  disc.def(
      "discretizeOffset",
      [](ampc::Discretizer& self, const Eigen::Ref<const Eigen::VectorXd>& wc) {
        Eigen::VectorXd wd{self.getStateDim()};
        self.discretizeOffset(wc, wd);
        return wd;
      },
      R"doc(
Discretize only a model offset with the state matrix and time step of the last
discretize() call: wd = G wc. Costs one matrix-vector product, with no matrix
exponential.

Args:
    wc: Continuous-time model offset (size state_dim).

Returns:
    wd: Discrete-time model offset.

Raises:
    ValueError: If wc has the wrong size.
    RuntimeError: If discretize() has not been called.
)doc",
      py::arg("wc"));

  disc.def("hasDiscretized", &ampc::Discretizer::hasDiscretized,
           "Whether discretize() has been called.");
  disc.def("getExponentialIntegral", &ampc::Discretizer::getExponentialIntegral,
           py::return_value_policy::copy, R"doc(
Get a copy of G = integral_0^dt exp(Ac s) ds from the last discretize() call
(Bd = G Bc, wd = G wc).

Raises:
    RuntimeError: If discretize() has not been called.
)doc");
  disc.def("getStateDim", &ampc::Discretizer::getStateDim,
           "Get the state dimension.");
  disc.def("getInputDim", &ampc::Discretizer::getInputDim,
           "Get the input dimension.");
  disc.def("__repr__", [](const ampc::Discretizer& self) {
    std::ostringstream oss;
    oss << "Discretizer(state_dim=" << self.getStateDim()
        << ", input_dim=" << self.getInputDim() << ')';
    return oss.str();
  });
}

} // namespace affine_mpc_py
