#include "affine_mpc_py_module.hpp"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <sstream>

#include "affine_mpc/options.hpp"

namespace affine_mpc_py {
namespace ampc = affine_mpc;
namespace py = pybind11;

void moduleAddOptions(py::module& m)
{
  py::class_<ampc::Options> opt(m, "Options", R"doc(
Controls which optional features are enabled at MPC construction time.

All flags default to false (bounded_states to empty, every state, and the
constraint counts to zero). These options are immutable after constructing an
MPC instance.

Attributes:
    use_input_cost: Enables input regularization term in the cost function
        ((uref_k - u_k)^T R (uref_k - u_k)).
    slew_initial_input: Slew-rate constraint on initial input
        (|u0 - u_prev| <= u0_slew).
    slew_control_points: Enables slew-rate constraints on parameterization
        control points (|v_{i+1} - v_i| <= control_point_slew).
    saturate_states: Enables state saturation constraints.
    saturate_input_trajectory: Enables saturation of the sampled input
        trajectory rather than just the control points. Ignored for degree 0,
        where the control points are the sampled inputs. For degree 1 only the
        samples adjacent to each knot are constrained (at most
        2*num_control_points-2 rows), which is exact for non-integer knots. For
        degree > 1 every sample is constrained. Allows control points to be
        outside of input limits while keeping inputs within limits.
    use_input_rate_cost: Enables a cost on the change of the sampled inputs
        ((u_k - u_{k-1})^T R_rate (u_k - u_{k-1}) with u_{-1} = u_prev).
    soft_state_bounds: Makes the state limits soft (requires
        saturate_states): x_min - s <= x_k <= x_max + s with one slack s >= 0
        per bounded state, penalized by l1^T s + s^T diag(l2) s.
    bounded_states: Indices of the states with limits (requires
        saturate_states). Empty (the default) bounds every state.
    num_general_constraints: Number p of general linear constraints
        g_min <= E x_k + F u_k <= g_max at every step k = 0, ..., T-1 (x_0 is
        the measured state). Zero (the default) disables them.
    num_terminal_constraints: Number of terminal constraints
        gf_min <= Ef x_T <= gf_max. Zero (the default) disables them.
                          )doc");

  opt.def(
      py::init([](const bool use_input_cost, const bool slew_initial_input,
                  const bool slew_control_points, const bool saturate_states,
                  const bool saturate_input_trajectory,
                  const bool use_input_rate_cost, const bool soft_state_bounds,
                  const std::vector<int>& bounded_states,
                  const int num_general_constraints,
                  const int num_terminal_constraints) {
        ampc::Options opts;
        opts.use_input_cost = use_input_cost;
        opts.slew_initial_input = slew_initial_input;
        opts.slew_control_points = slew_control_points;
        opts.saturate_states = saturate_states;
        opts.saturate_input_trajectory = saturate_input_trajectory;
        opts.use_input_rate_cost = use_input_rate_cost;
        opts.soft_state_bounds = soft_state_bounds;
        opts.bounded_states = bounded_states;
        opts.num_general_constraints = num_general_constraints;
        opts.num_terminal_constraints = num_terminal_constraints;
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
        control points (|v_{i+1} - v_i| <= control_point_slew).
    saturate_states: Enables state saturation constraints.
    saturate_input_trajectory: Enables saturation of the sampled input
        trajectory rather than just the control points. Ignored for degree 0,
        where the control points are the sampled inputs. For degree 1 only the
        samples adjacent to each knot are constrained (at most
        2*num_control_points-2 rows), which is exact for non-integer knots. For
        degree > 1 every sample is constrained. Allows control points to be
        outside of input limits while keeping inputs within limits.
    use_input_rate_cost: Enables a cost on the change of the sampled inputs
        ((u_k - u_{k-1})^T R_rate (u_k - u_{k-1}) with u_{-1} = u_prev).
    soft_state_bounds: Makes the state limits soft (requires
        saturate_states): x_min - s <= x_k <= x_max + s with one slack s >= 0
        per bounded state, penalized by l1^T s + s^T diag(l2) s.
    bounded_states: Indices of the states with limits (requires
        saturate_states). Empty (the default) bounds every state.
    num_general_constraints: Number p of general linear constraints
        g_min <= E x_k + F u_k <= g_max at every step k = 0, ..., T-1 (x_0 is
        the measured state). Zero (the default) disables them.
    num_terminal_constraints: Number of terminal constraints
        gf_min <= Ef x_T <= gf_max. Zero (the default) disables them.
      )doc",
      py::arg("use_input_cost") = false, py::arg("slew_initial_input") = false,
      py::arg("slew_control_points") = false,
      py::arg("saturate_states") = false,
      py::arg("saturate_input_trajectory") = false,
      py::arg("use_input_rate_cost") = false,
      py::arg("soft_state_bounds") = false,
      py::arg("bounded_states") = std::vector<int>{},
      py::arg("num_general_constraints") = 0,
      py::arg("num_terminal_constraints") = 0);

  opt.def_readwrite("use_input_cost", &ampc::Options::use_input_cost);
  opt.def_readwrite("slew_initial_input", &ampc::Options::slew_initial_input);
  opt.def_readwrite("slew_control_points", &ampc::Options::slew_control_points);
  opt.def_readwrite("saturate_states", &ampc::Options::saturate_states);
  opt.def_readwrite("saturate_input_trajectory",
                    &ampc::Options::saturate_input_trajectory);
  opt.def_readwrite("use_input_rate_cost", &ampc::Options::use_input_rate_cost);
  opt.def_readwrite("soft_state_bounds", &ampc::Options::soft_state_bounds);
  // assign a new list (a copy is returned, so appending to it has no effect)
  opt.def_readwrite("bounded_states", &ampc::Options::bounded_states);
  opt.def_readwrite("num_general_constraints",
                    &ampc::Options::num_general_constraints);
  opt.def_readwrite("num_terminal_constraints",
                    &ampc::Options::num_terminal_constraints);

  opt.def("__str__", [](const affine_mpc::Options& opts) {
    std::ostringstream oss;
    const bool capitalize_bools{true};
    print(oss, opts, capitalize_bools);
    return oss.str();
  });

  opt.def("__repr__", [](const affine_mpc::Options& opts) {
    std::ostringstream oss;
    oss << "Options(";
    bool prev{false};
    auto addField = [&](bool field, const std::string& name) {
      if (field) {
        oss << (prev ? ", " : "") << name << "=True";
        prev = true;
      }
    };
    addField(opts.use_input_cost, "use_input_cost");
    addField(opts.slew_initial_input, "slew_initial_input");
    addField(opts.slew_control_points, "slew_control_points");
    addField(opts.saturate_states, "saturate_states");
    addField(opts.saturate_input_trajectory, "saturate_input_trajectory");
    addField(opts.use_input_rate_cost, "use_input_rate_cost");
    addField(opts.soft_state_bounds, "soft_state_bounds");
    if (!opts.bounded_states.empty()) {
      oss << (prev ? ", " : "") << "bounded_states=[";
      for (size_t i{0}; i < opts.bounded_states.size(); ++i)
        oss << (i ? ", " : "") << opts.bounded_states[i];
      oss << "]";
      prev = true;
    }
    auto addCount = [&](int field, const std::string& name) {
      if (field != 0) {
        oss << (prev ? ", " : "") << name << '=' << field;
        prev = true;
      }
    };
    addCount(opts.num_general_constraints, "num_general_constraints");
    addCount(opts.num_terminal_constraints, "num_terminal_constraints");
    oss << ")";
    return oss.str();
  });
}

} // namespace affine_mpc_py
