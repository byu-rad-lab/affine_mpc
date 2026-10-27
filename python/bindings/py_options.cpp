#include "affine_mpc_py_module.hpp"

#include <pybind11/native_enum.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <sstream>

#include "affine_mpc/options.hpp"

namespace affine_mpc_py {
namespace ampc = affine_mpc;
namespace py = pybind11;

void moduleAddOptions(py::module& m)
{
  py::native_enum<ampc::StateBoundMode>(m, "StateBoundMode", "enum.Enum",
                                        "Whether and how the MPC bounds the "
                                        "predicted states, chosen with "
                                        "Options.state_bounds.")
      .value("Off", ampc::StateBoundMode::Off, "No state bounds (the default).")
      .value("Hard", ampc::StateBoundMode::Hard,
             "Hard bounds x_min <= x_k <= x_max on the bounded states at every "
             "step k = 1, ..., T (set with MPC.setStateLimits()).")
      .value("Soft", ampc::StateBoundMode::Soft,
             "Soft bounds: one slack s >= 0 per bounded state, shared over the "
             "horizon, may widen both of its limits, "
             "x_min - s <= x_k <= x_max + s, at the cost "
             "l1^T s + s^T diag(l2) s (set with MPC.setStateLimits() and "
             "MPC.setStateBoundPenalties()).")
      .finalize();

  py::class_<ampc::Options> opt(m, "Options", R"doc(
The optional costs and constraints of the MPC problem, chosen at construction.

Every optional cost and constraint is off by default. The options fix the layout
of the QP, so they can not be changed after constructing an MPC instance. The
fields are grouped as follows.

Costs:
    use_input_cost: Enables the input cost on the control points
        ((uref_k - u_k)^T R (uref_k - u_k)).
    use_input_rate_cost: Enables a cost on the change of the sampled inputs
        ((u_k - u_{k-1})^T R_rate (u_k - u_{k-1}) with u_{-1} = u_prev).

Input constraints (the input limits are always enforced):
    saturate_input_trajectory: Enables saturation of the sampled input
        trajectory rather than just the control points. Ignored for degree 0,
        where the control points are the sampled inputs. For degree 1 only the
        samples adjacent to each knot are constrained (at most
        2*num_control_points-2 rows), which is exact for non-integer knots. For
        degree > 1 every sample is constrained. Allows control points to be
        outside of input limits while keeping inputs within limits. A custom
        basis (Parameterization.fromBasis()) always constrains every sample.
    slew_initial_input: Slew-rate constraint on the initial input
        (|u0 - u_prev| <= u0_slew).
    slew_control_points: Slew-rate constraints on the parameterization control
        points (|v_{i+1} - v_i| <= control_point_slew). Requires a B-spline
        parameterization.

State constraints:
    state_bounds: StateBoundMode.Off (the default), Hard, or Soft state bounds.
    bounded_states: Indices of the states with limits (requires state_bounds
        other than Off). Empty (the default) bounds every state.

General constraints:
    num_general_constraints: Number p of general linear constraints
        h_min <= Hx x_k + Hu u_k <= h_max at every step k = 0, ..., T-1 (x_0 is
        the measured state). Zero (the default) disables them.
    num_terminal_constraints: Number of terminal constraints
        hf_min <= Hf x_T <= hf_max. Zero (the default) disables them.
                          )doc");

  opt.def(
      py::init([](const bool use_input_cost, const bool use_input_rate_cost,
                  const bool saturate_input_trajectory,
                  const bool slew_initial_input, const bool slew_control_points,
                  const ampc::StateBoundMode state_bounds,
                  const std::vector<int>& bounded_states,
                  const int num_general_constraints,
                  const int num_terminal_constraints) {
        ampc::Options opts;
        opts.use_input_cost = use_input_cost;
        opts.use_input_rate_cost = use_input_rate_cost;
        opts.saturate_input_trajectory = saturate_input_trajectory;
        opts.slew_initial_input = slew_initial_input;
        opts.slew_control_points = slew_control_points;
        opts.state_bounds = state_bounds;
        opts.bounded_states = bounded_states;
        opts.num_general_constraints = num_general_constraints;
        opts.num_terminal_constraints = num_terminal_constraints;
        return opts;
      }),
      R"doc(
Construct an Options struct defining optional MPC configuration features. All
arguments are keyword-only.

Args:
    use_input_cost: Enables the input cost on the control points
        ((uref_k - u_k)^T R (uref_k - u_k)).
    use_input_rate_cost: Enables a cost on the change of the sampled inputs
        ((u_k - u_{k-1})^T R_rate (u_k - u_{k-1}) with u_{-1} = u_prev).
    saturate_input_trajectory: Enables saturation of the sampled input
        trajectory rather than just the control points (see Options).
    slew_initial_input: Slew-rate constraint on the initial input
        (|u0 - u_prev| <= u0_slew).
    slew_control_points: Slew-rate constraints on the parameterization control
        points (|v_{i+1} - v_i| <= control_point_slew). Requires a B-spline
        parameterization.
    state_bounds: StateBoundMode.Off (the default), Hard, or Soft state bounds.
    bounded_states: Indices of the states with limits (requires state_bounds
        other than Off). Empty (the default) bounds every state.
    num_general_constraints: Number p of general linear constraints
        h_min <= Hx x_k + Hu u_k <= h_max at every step k = 0, ..., T-1 (x_0 is
        the measured state). Zero (the default) disables them.
    num_terminal_constraints: Number of terminal constraints
        hf_min <= Hf x_T <= hf_max. Zero (the default) disables them.
      )doc",
      py::kw_only(), py::arg("use_input_cost") = false,
      py::arg("use_input_rate_cost") = false,
      py::arg("saturate_input_trajectory") = false,
      py::arg("slew_initial_input") = false,
      py::arg("slew_control_points") = false,
      py::arg("state_bounds") = ampc::StateBoundMode::Off,
      py::arg("bounded_states") = std::vector<int>{},
      py::arg("num_general_constraints") = 0,
      py::arg("num_terminal_constraints") = 0);

  opt.def_readwrite("use_input_cost", &ampc::Options::use_input_cost);
  opt.def_readwrite("use_input_rate_cost", &ampc::Options::use_input_rate_cost);
  opt.def_readwrite("saturate_input_trajectory",
                    &ampc::Options::saturate_input_trajectory);
  opt.def_readwrite("slew_initial_input", &ampc::Options::slew_initial_input);
  opt.def_readwrite("slew_control_points", &ampc::Options::slew_control_points);
  opt.def_readwrite("state_bounds", &ampc::Options::state_bounds);
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

  // the options that differ from the defaults
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
    addField(opts.use_input_rate_cost, "use_input_rate_cost");
    addField(opts.saturate_input_trajectory, "saturate_input_trajectory");
    addField(opts.slew_initial_input, "slew_initial_input");
    addField(opts.slew_control_points, "slew_control_points");
    if (opts.state_bounds != ampc::StateBoundMode::Off) {
      oss << (prev ? ", " : "") << "state_bounds=StateBoundMode."
          << opts.state_bounds;
      prev = true;
    }
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

  py::native_enum<ampc::ModelSparsity>(
      m, "ModelSparsity", "enum.Enum",
      "Which entries of the discrete model Ad and Bd the solver tracks, chosen "
      "with MPC.initializeSolver(). OSQP fixes the sparsity pattern at "
      "initialization, and later updates only change the values of the "
      "tracked entries: a model entry outside them must stay zero (a nonzero "
      "one is silently dropped). Tracking fewer entries makes each solve "
      "cheaper when the model has structural zeros.")
      .value("InitialNonzeros", ampc::ModelSparsity::InitialNonzeros,
             "Track the entries of Ad and Bd that are nonzero when "
             "initializeSolver() is called (the default). Entries that may "
             "become nonzero later must be nonzero then, e.g. by setting a "
             "representative model before it and the actual model after it.")
      .value(
          "Full", ampc::ModelSparsity::Full,
          "Track every entry of Ad and Bd, so any entry may change between "
          "zero and nonzero later. Costs more per solve when the model has "
          "structural zeros. Supported by the sparse formulation only for now.")
      .finalize();
}

} // namespace affine_mpc_py
