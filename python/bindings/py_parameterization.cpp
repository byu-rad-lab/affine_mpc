#include "affine_mpc_py_module.hpp"

#include <Eigen/Core>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <sstream>

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
    makeClampedKnots: Generate a uniform or geometric knot vector to modify.
    makeUniformClampedKnots: Generate a knot vector that can be modified.
    moveBlocking: Factory method for move-blocking parameterization.
    linearInterp: Factory method for linear interpolation parameterization.
    bspline: Factory method for clamped B-spline parameterization.
                                         )doc");

  param.def_static("moveBlocking",
                   py::overload_cast<int, int, double>(
                       &ampc::Parameterization::moveBlocking),
                   R"doc(
Factory method for move-blocking parameterization with uniform or
geometrically spaced blocks.

Args:
    horizon_steps: Number of discrete steps in the MPC horizon.
    num_control_points: Number of B-spline control points. Cannot be greater
        than horizon_steps.
    knot_ratio: Ratio of consecutive knot interval lengths (positive): the
        intervals are proportional to 1, knot_ratio, knot_ratio^2, ... and
        span [0, horizon_steps - 1]. The default of 1 gives uniform knots. A
        ratio above 1 puts short intervals (more flexibility) at the start of
        the horizon and a ratio below 1 puts them at the end.

Raises:
    ValueError: If the blocks are so short that some block has no sample of
        its own (the sampled basis would not have full column rank), or if
        any argument is invalid.

returns:
    out: Parameterization instance with move-blocking structure.
)doc",
                   py::arg("horizon_steps"), py::arg("num_control_points"),
                   py::arg("knot_ratio") = 1.0);

  param.def_static(
      "moveBlocking",
      py::overload_cast<int, const Eigen::Ref<const Eigen::VectorXd>&>(
          &ampc::Parameterization::moveBlocking),
      R"doc(
Factory method for move-blocking parameterization with custom change points.

Args:
    horizon_steps: Number of discrete steps in the MPC horizon.
    change_points: Increasing start times of the blocks. The first must be 0
        and the last at most horizon_steps - 1. horizon_steps - 1 is appended
        to close the last block, so 0, 1, ..., horizon_steps - 1 gives one
        control point per step.

returns:
    out: Parameterization instance with move-blocking structure.
)doc",
      py::arg("horizon_steps"), py::arg("change_points"));

  param.def_static("linearInterp",
                   py::overload_cast<int, int, double>(
                       &ampc::Parameterization::linearInterp),
                   R"doc(
Factory method for linear interpolation parameterization with uniform or
geometrically spaced endpoints.

Args:
    horizon_steps: Number of discrete steps in the MPC horizon.
    num_control_points: Number of B-spline control points. Cannot be greater
        than horizon_steps.
    knot_ratio: Ratio of consecutive knot interval lengths (positive): the
        intervals are proportional to 1, knot_ratio, knot_ratio^2, ... and
        span [0, horizon_steps - 1]. The default of 1 gives uniform knots. A
        ratio above 1 puts short intervals (more flexibility) at the start of
        the horizon and a ratio below 1 puts them at the end.

Raises:
    ValueError: If the segments are so short that the sampled basis does not
        have full column rank, or if any argument is invalid.

returns:
    out: Parameterization instance with linear interpolation structure.
)doc",
                   py::arg("horizon_steps"), py::arg("num_control_points"),
                   py::arg("knot_ratio") = 1.0);

  param.def_static(
      "linearInterp",
      py::overload_cast<int, const Eigen::Ref<const Eigen::VectorXd>&>(
          &ampc::Parameterization::linearInterp),
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

  param.def_static("bspline",
                   py::overload_cast<int, int, int, double>(
                       &ampc::Parameterization::bspline),
                   R"doc(
Factory method for clamped B-spline parameterization with uniform or
geometrically spaced active knots.

Args:
    horizon_steps: Number of discrete steps in the MPC horizon.
    degree: Degree of B-spline polynomials. Must be less than num_control_points.
    num_control_points: Number of B-spline control points. Cannot be greater
        than horizon_steps.
    knot_ratio: Ratio of consecutive knot interval lengths (positive): the
        intervals are proportional to 1, knot_ratio, knot_ratio^2, ... and
        span [0, horizon_steps - 1]. The default of 1 gives uniform knots. A
        ratio above 1 puts short intervals (more flexibility) at the start of
        the horizon and a ratio below 1 puts them at the end.

Raises:
    ValueError: If the knot intervals are so short that the sampled basis
        does not have full column rank, or if any argument is invalid.

returns:
    out: Parameterization instance with clamped B-spline structure.
)doc",
                   py::arg("horizon_steps"), py::arg("degree"),
                   py::arg("num_control_points"), py::arg("knot_ratio") = 1.0);

  param.def_static(
      "bspline",
      py::overload_cast<int, int, const Eigen::Ref<const Eigen::VectorXd>&>(
          &ampc::Parameterization::bspline),
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
        horizon_steps+degree+1]. Must be non-decreasing. The first active
        knot (index degree) must be 0 and the last active knot (index
        len(knots)-degree-1) must be horizon_steps-1; for unclamped splines
        the knots outside them extend beyond [0, horizon_steps-1]. Knots
        among the first and last degree+1 are snapped to 0 and
        horizon_steps-1 when within 1e-9 * horizon_steps (round-off). The
        basis sampled at k = 0, ..., horizon_steps-1 must have full column
        rank: every control point needs its own sample, in increasing order,
        at which its basis function is nonzero (Schoenberg-Whitney). Repeated
        knots are allowed, except an interior knot equal to the first active
        knot or, for degree > 0, the last.
          )doc",
            py::arg("horizon_steps"), py::arg("degree"), py::arg("knots"));

  param.def("evaluate", &ampc::Parameterization::evaluate,
            R"doc(
Evaluate an input trajectory from provided control_points.

Args:
    input_dim: Dimension of input vector.
    control_points (vector): Parameterized input trajectory as a vector of size
        input_dim*num_control_points.

Returns:
    input_traj (vector): The evaluated input trajectory.
            )doc",
            py::arg("input_dim"), py::arg("control_points"));

  param.def_static("makeClampedKnots",
                   &ampc::Parameterization::makeClampedKnots,
                   R"doc(
Static method for generating a clamped knot vector with uniform or
geometrically spaced active knots (the knots used by the factories). Useful for
generating a starting point that can be modified for custom parameterizations.

Args:
    horizon_steps: Number of discrete steps in the MPC horizon.
    degree: Degree of B-spline polynomials. Must be less than num_control_points.
    num_control_points: Number of B-spline control points. Cannot be greater
        than horizon_steps.
    knot_ratio: Ratio of consecutive knot interval lengths (positive): the
        intervals are proportional to 1, knot_ratio, knot_ratio^2, ... and
        span [0, horizon_steps - 1]. The default of 1 gives uniform knots. A
        ratio above 1 puts short intervals (more flexibility) at the start of
        the horizon and a ratio below 1 puts them at the end.

Raises:
    ValueError: If the knot intervals are so short that the sampled basis
        does not have full column rank, or if any argument is invalid.

Returns:
    knots: The clamped knot vector.
)doc",
                   py::arg("horizon_steps"), py::arg("degree"),
                   py::arg("num_control_points"), py::arg("knot_ratio") = 1.0);

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
  param.def_readonly("band_width", &ampc::Parameterization::band_width,
                     "Number of consecutive control points that can affect "
                     "one sampled input (degree + 1 for B-splines).");
  param.def_readonly("band_start", &ampc::Parameterization::band_start,
                     "Index of the first control point that affects the input "
                     "at each step k (size horizon_steps).");
  param.def_readonly("band_weights", &ampc::Parameterization::band_weights,
                     "Basis weights in banded form (band_width x "
                     "horizon_steps). The input at step k is "
                     "sum_i band_weights[i, k] * control_point[band_start[k] "
                     "+ i].");

  param.def("__str__", [](const ampc::Parameterization& self) {
    std::ostringstream os;
    os << self;
    return os.str();
  });

  param.def("__repr__", [](const ampc::Parameterization& self) {
    const Eigen::VectorXd uniform_knots{
        ampc::Parameterization::makeUniformClampedKnots(
            self.horizon_steps, self.degree, self.num_control_points)};
    std::ostringstream oss;
    oss << "Parameterization(horizon_steps=" << self.horizon_steps
        << ", degree=" << self.degree;

    if (self.knots.isApprox(uniform_knots))
      oss << ", num_control_points=" << self.num_control_points;
    else {
      const Eigen::IOFormat fmt{
          Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "[", "]"};
      oss << ", knots=" << self.knots.transpose().format(fmt);
    }
    oss << ')';
    return oss.str();
  });
}

} // namespace affine_mpc_py
