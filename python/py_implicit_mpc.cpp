#include "affine_mpc_py_module.hpp"
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include "affine_mpc/implicit_mpc.hpp"

namespace py = pybind11;

void moduleAddImplicitMPC(py::module& m)
{
  py::class_<ImplicitMPC, MPCBase> impc(m, "ImplicitMPC", "Implicit MPC class");
  impc.def(py::init<const int, const int, const int, const int, const bool, const bool,
           const bool>(), "Constructor", py::arg("num_states"), py::arg("num_inputs"),
           py::arg("horizon_length"), py::arg("num_knot_points"),
           py::arg("use_input_cost")=false, py::arg("use_slew_rate")=false,
           py::arg("saturate_states")=false);

  impc.def("getPredictedStateTrajectory",
    [](ImplicitMPC& self, Ref<VectorXd> x_traj)
    {
      self.getPredictedStateTrajectory(x_traj);
      return x_traj;
    }, "Get predicted state trajectory from previous solve", py::arg("x_traj"));
  impc.def("getPredictedStateTrajectory",
    [](ImplicitMPC& self)
    {
      VectorXd x_traj{self.getNumStates()*self.getHorizonLength()};
      self.getPredictedStateTrajectory(x_traj);
      return x_traj;
    }, "Calculate optimal parameterized trajectory of inputs");

  impc.def("setInputLimits", &ImplicitMPC::setInputLimits, "Set input saturation limits",
    py::arg("u_min"), py::arg("u_max"));
  impc.def("setStateLimits", &ImplicitMPC::setStateLimits, "Set state saturation limits",
    py::arg("x_min"), py::arg("x_max"));
  impc.def("setSlewRate", &ImplicitMPC::setSlewRate, "Set slew rate constraint limits",
    py::arg("u_slew"));
}
