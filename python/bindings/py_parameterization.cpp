#include "affine_mpc_py_module.hpp"

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

#include "affine_mpc/parameterization.hpp"

namespace affine_mpc_py {
namespace ampc = affine_mpc;
namespace py = pybind11;

void moduleAddParameterization(py::module& m)
{
  py::class_<ampc::Parameterization> param(m, "Parameterization",
                                           R"doc(
Encapsulates B-spline input trajectory parameterization for MPC.

Provides constructors and factory methods for generating knot vectors and
metadata to map control points to per-step input trajectories over the
prediction horizon.

The preferred usage is via the static factory methods for common
parameterizations.

Attributes:
    horizon_steps: Number of discrete steps in the MPC horizon.
    degree: Degree of B-spline polynomials.
    num_control_points: Number of B-spline control points.
    knots: Full knot vector of size num_control_points + degree + 1.

Static Methods:
    makeUniformClampedKnots: Generate a knot vector that can be modified.
    moveBlocking: Factory method for move-blocking parameterization.
    linearInterp: Factory method for linear interpolation parameterization.
    bspline: Factory method for clamped B-spline parameterization.
                                         )doc");

  param.def_static(
      "moveBlocking",
      [](int horizon_steps, int num_control_points) {
        return ampc::Parameterization::moveBlocking(horizon_steps,
                                                    num_control_points);
      },
      R"doc(
Factory method for uniform move-blocking parameterization.

Args:
    horizon_steps: Number of discrete steps in the MPC horizon.
    num_control_points: Number of B-spline control points. Cannot be greater
        than horizon_steps.

returns:
    out: Parameterization instance with uniform move-blocking structure.
)doc",
      py::arg("horizon_steps"), py::arg("num_control_points"));

  param.def_static(
      "moveBlocking",
      [](int horizon_steps,
         const Eigen::Ref<const Eigen::VectorXd>& change_points) {
        return ampc::Parameterization::moveBlocking(horizon_steps,
                                                    change_points);
      },
      R"doc(
Factory method for move-blocking parameterization with custom change points.

Args:
    horizon_steps: Number of discrete steps in the MPC horizon.
    change_points: Vector of change point locations.

returns:
    out: Parameterization instance with move-blocking structure.
)doc",
      py::arg("horizon_steps"), py::arg("change_points"));

  param.def_static(
      "linearInterp",
      [](int horizon_steps, int num_control_points) {
        return ampc::Parameterization::linearInterp(horizon_steps,
                                                    num_control_points);
      },
      R"doc(
Factory method for uniform linear interpolation parameterization.

Args:
    horizon_steps: Number of discrete steps in the MPC horizon.
    num_control_points: Number of B-spline control points. Cannot be greater
        than horizon_steps.

returns:
    out: Parameterization instance with uniform linear interpolation structure.
)doc",
      py::arg("horizon_steps"), py::arg("num_control_points"));

  param.def_static(
      "linearInterp",
      [](int horizon_steps,
         const Eigen::Ref<const Eigen::VectorXd>& endpoints) {
        return ampc::Parameterization::linearInterp(horizon_steps, endpoints);
      },
      R"doc(
Factory method for linear interpolation parameterization with custom endpoints.

Args:
    horizon_steps: Number of discrete steps in the MPC horizon.
    endpoints: Vector of linear segment endpoint locations. Must include 0 and
        horizon_steps - 1.

returns:
    out: Parameterization instance with linear interpolation structure.
)doc",
      py::arg("horizon_steps"), py::arg("endpoints"));

  param.def_static(
      "bspline",
      [](int horizon_steps, int degree, int num_control_points) {
        return ampc::Parameterization::bspline(horizon_steps, degree,
                                               num_control_points);
      },
      R"doc(
Factory method for uniform clamped B-spline parameterization.

Args:
    horizon_steps: Number of discrete steps in the MPC horizon.
    degree: Degree of B-spline polynomials. Must be less than num_control_points.
    num_control_points: Number of B-spline control points. Cannot be greater
        than horizon_steps.

returns:
    out: Parameterization instance with uniform clamped B-spline structure.
)doc",
      py::arg("horizon_steps"), py::arg("degree"),
      py::arg("num_control_points"));

  param.def_static(
      "bspline",
      [](int horizon_steps, int degree,
         const Eigen::Ref<const Eigen::VectorXd>& active_knots) {
        return ampc::Parameterization::bspline(horizon_steps, degree,
                                               active_knots);
      },
      R"doc(
Factory method for clamped B-spline parameterization with custom active knots.

Args:
    horizon_steps: Number of discrete steps in the MPC horizon.
    degree: Degree of B-spline polynomials. Must be less than horizon_steps.
    active_knots: Vector of active knots (need at least deg+1).

returns:
    out: Parameterization instance with clamped B-spline structure.
)doc",
      py::arg("horizon_steps"), py::arg("degree"), py::arg("active_knots"));

  param.def(py::init<const int, const int, const int>(),
            R"doc(
Direct constructor for uniform clamped B-spline parameterization.

Args:
    horizon_steps: Number of discrete steps in the MPC horizon.
    degree: Degree of B-spline polynomials. Must be less than num_control_points.
    num_control_points: Number of B-spline control points. Cannot be greater
        than horizon_steps.
          )doc",
            py::arg("horizon_steps"), py::arg("degree"),
            py::arg("num_control_points"));

  param.def(py::init<const int, const int,
                     const Eigen::Ref<const Eigen::VectorXd>&>(),
            R"doc(
Direct constructor for advanced use cases with custom knot vector
(e.g. unclamped B-splines).

Args:
    horizon_steps: Number of discrete steps in the MPC horizon.
    degree: Degree of B-spline polynomials. Must be less than horizon_steps.
    knots: Full knot vector with size in the range [2*(degree+1),
        horizon_steps+degree+1]. Must be non-decreasing. First knot must be 0
        and last knot must be horizon_steps-1.
          )doc",
            py::arg("horizon_steps"), py::arg("degree"), py::arg("knots"));

  param.def("evaluate", &ampc::Parameterization::evaluate,
            R"doc(
Evalutate an input trajectory from provided control_points.

Args:
    input_dim: Dimension of input vector.
    control_points (vector): Parameterized input trajectory as a vector of size
        input_dim*num_control_points.

Returns:
    input_traj (vector): The evaluated input trajectory.
            )doc",
            py::arg("input_dim"), py::arg("control_points"));

  param.def_static("makeUniformClampedKnots",
                   &ampc::Parameterization::makeUniformClampedKnots,
                   R"doc(
Static method for generating a uniform clamped knot vector. Useful for
generating a starting point that can be modified for custom parameterizations.

Args:
    horizon_steps: Number of discrete steps in the MPC horizon.
    degree: Degree of B-spline polynomials. Must be less than num_control_points.
    num_control_points: Number of B-spline control points. Cannot be greater
        than horizon_steps.

Returns:
    knots: The uniform clamped knot vector.
                 )doc",
                   py::arg("horizon_steps"), py::arg("degree"),
                   py::arg("num_control_points"));

  param.def_readonly("horizon_steps", &ampc::Parameterization::horizon_steps);
  param.def_readonly("degree", &ampc::Parameterization::degree);
  param.def_readonly("num_control_points",
                     &ampc::Parameterization::num_control_points);
  param.def_readonly("knots", &ampc::Parameterization::knots);
}

} // namespace affine_mpc_py
