#include "affine_mpc_py_module.hpp"

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

#include "affine_mpc/bspline_mpc.hpp"

namespace affine_mpc {
namespace py = pybind11;


void moduleAddBSplineMPC(py::module& m)
{
  py::class_<BSplineMPC, MPCBase> impc(m, "BSplineMPC", "BSpline MPC class");
  impc.def(py::init<const int, const int, const int, const int, const int,
                    const Eigen::Ref<const Eigen::VectorXd>&, const bool,
                    const bool, const bool>(),
           R"(
Constructor.

Args:
    num_states (int): Number of states in the system.
    num_inputs (int): Number of inputs in the system.
    len_horizon (int): Length of the prediction horizon.
    num_control_points (int): Number of control points used to parameterize
        the input trajectory.
    spline_degree (int): Degree of the polynomial segments of the input
        trajectory spline.
    knots (NDArray[np.float64]): Spline's knot point vector. Size must be equal
        to num_control_points - spline_degree + 1. Must be non-decreasing.
        If empty, uniform knots will be used.
    use_input_cost (bool): Whether to use the input cost. (uk.T @ R @ uk)
    use_slew_rate (bool): Whether to use a slew rate constraint.
        (|ctrl_next - ctrl_prev| <= slew)
    saturate_states (bool): Whether to saturate states. (x_min <= x_k <= x_max)
           )",
           py::arg("num_states"), py::arg("num_inputs"), py::arg("len_horizon"),
           py::arg("num_control_points"), py::arg("spline_degree"),
           py::arg("knots") = Eigen::VectorXd(0),
           py::arg("use_input_cost") = false, py::arg("use_slew_rate") = false,
           py::arg("saturate_states") = false);
  //   impc.def(py::init<const int, const int, const int, const int, const int,
  //                     const Eigen::Ref<const Eigen::VectorXd>&, const bool,
  //                     const bool, const bool>(),
  //            R"(
  // Constructor.
  //
  // Args:
  //     num_states (int): Number of states in the system.
  //     num_inputs (int): Number of inputs in the system.
  //     len_horizon (int): Length of the prediction horizon.
  //     num_control_points (int): Number of control points used to parameterize
  //         the input trajectory.
  //     spline_degree (int): Degree of the polynomial segments of the input
  //         trajectory spline.
  //     knots (NDArray[np.float64]): Spline's knot vector. Can have length of
  //     num_control_points use_input_cost (bool): Whether to use the input
  //     cost. (uk.T @ R @ uk) use_slew_rate (bool): Whether to use a slew rate
  //     constraint.
  //         (|ctrl_next - ctrl_prev| <= slew)
  //     saturate_states (bool): Whether to saturate states. (x_min <= x_k <=
  //     x_max)
  //            )",
  //            py::arg("num_states"), py::arg("num_inputs"),
  //            py::arg("len_horizon"), py::arg("num_control_points"),
  //            py::arg("spline_degree"), py::arg("knots"),
  //            py::arg("use_input_cost") = false, py::arg("use_slew_rate") =
  //            false, py::arg("saturate_states") = false);

  impc.def(
      "getInputTrajectory",
      [](BSplineMPC& self, Eigen::Ref<Eigen::VectorXd> u_traj) {
        self.getInputTrajectory(u_traj);
        return u_traj;
      },
      R"(
Get optimal trajectory of inputs from previous solve evaluated at each step in the prediction horizon.

Args:
    u_traj (NDArray[numpy.float64]): Array in which to store the trajectory (same memory as output).
Return:
    u_traj (NDArray[numpy.float64]): Optimized input trajectory (same memory as function argument)
      )",
      py::arg("u_traj"));
  impc.def(
      "getInputTrajectory",
      [](BSplineMPC& self) {
        Eigen::VectorXd u_traj{self.getNumInputs() * self.getHorizonLength()};
        self.getInputTrajectory(u_traj);
        return u_traj;
      },
      R"(
Get optimal trajectory of inputs from previous solve evaluated at each step in the prediction horizon.

Return:
    u_traj (NDArray[numpy.float64]): Optimized input trajectory (same memory as function argument)
      )");

  impc.def(
      "getPredictedStateTrajectory",
      [](BSplineMPC& self, Eigen::Ref<Eigen::VectorXd> x_traj) {
        self.getPredictedStateTrajectory(x_traj);
        return x_traj;
      },
      "Get predicted state trajectory from previous solve", py::arg("x_traj"));
  impc.def(
      "getPredictedStateTrajectory",
      [](BSplineMPC& self) {
        Eigen::VectorXd x_traj{self.getNumStates() * self.getHorizonLength()};
        self.getPredictedStateTrajectory(x_traj);
        return x_traj;
      },
      "Get predicted state trajectory from previous solve");

  impc.def("setInputLimits", &BSplineMPC::setInputLimits,
           "Set input saturation limits", py::arg("u_min"), py::arg("u_max"));
  impc.def("setStateLimits", &BSplineMPC::setStateLimits,
           "Set state saturation limits", py::arg("x_min"), py::arg("x_max"));
  impc.def("setSlewRate", &BSplineMPC::setSlewRate,
           "Set slew rate constraint limits", py::arg("u_slew"));
}

} // namespace affine_mpc
