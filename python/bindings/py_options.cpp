#include "affine_mpc_py_module.hpp"

#include <pybind11/pybind11.h>

#include "affine_mpc/options.hpp"

namespace affine_mpc_py {
namespace ampc = affine_mpc;
namespace py = pybind11;

void moduleAddOptions(py::module& m)
{
  py::class_<ampc::Options> opt(m, "Options", R"doc(
Controls which optional features are enabled at MPC construction time.

All fields default to false. These options are immutable after constructing an
MPC instance.

Attributes:
    use_input_cost: Enables input regularization term in the cost function
        ((uref_k - u_k)^T R (uref_k - u_k)).
    slew_initial_input: Slew-rate constraint on initial input
        (|u0 - u_prev| <= u0_slew).
    slew_control_points: Enables slew-rate constraints on parameterization
        control points (|v_{i+1} - v_i| <= u_slew).
    saturate_states: Enables state saturation constraints.
    saturate_input_trajectory: Enables saturation of each input in the
        trajectory rather than just the control points. Only applicable for
        parameterizations with degree > 1. This adds constraints to the
        optimiztion, but can allow control points to be outside of input limits
        while keeping inputs within limits.
                          )doc");

  opt.def(
      py::init([](const bool use_input_cost, const bool slew_initial_input,
                  const bool slew_control_points, const bool saturate_states,
                  const bool saturate_input_trajectory) {
        ampc::Options opts;
        opts.use_input_cost = use_input_cost;
        opts.slew_initial_input = slew_initial_input;
        opts.slew_control_points = slew_control_points;
        opts.saturate_states = saturate_states;
        opts.saturate_input_trajectory = saturate_input_trajectory;
        return opts;
      }),
      R"doc(
Construct an Options struct defining optional MPC configuration features.

Args:
    use_input_cost: Enables input regularization term in the cost function
        ((uref_k - u_k)^T R (uref_k - u_k)).
    slew_initial_input: Slew-rate constraint on initial input
        (|u0 - u_prev| <= u0_slew).
    slew_control_points: Enables slew-rate constraints on parameterization
        control points (|v_{i+1} - v_i| <= u_slew).
    saturate_states: Enables state saturation constraints.
    saturate_input_trajectory: Enables saturation of each input in the
        trajectory rather than just the control points. Only applicable for
        parameterizations with degree > 1. This adds constraints to the
        optimiztion, but can allow control points to be outside of input limits
        while keeping inputs within limits.
      )doc",
      py::arg("use_input_cost") = false, py::arg("slew_initial_input") = false,
      py::arg("slew_control_points") = false,
      py::arg("saturate_states") = false,
      py::arg("saturate_input_trajectory") = false);

  opt.def_readwrite("use_input_cost", &ampc::Options::use_input_cost);
  opt.def_readwrite("slew_initial_input", &ampc::Options::slew_initial_input);
  opt.def_readwrite("slew_control_points", &ampc::Options::slew_control_points);
  opt.def_readwrite("saturate_states", &ampc::Options::saturate_states);
  opt.def_readwrite("saturate_input_trajectory",
                    &ampc::Options::saturate_input_trajectory);
}

} // namespace affine_mpc_py
