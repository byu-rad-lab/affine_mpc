#include "affine_mpc_py_module.hpp"

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

#include "affine_mpc/implicit_mpc.hpp"

namespace affine_mpc {
namespace py = pybind11;


void moduleAddImplicitMPC(py::module& m)
{
  py::class_<ImplicitMPC, MPCBase> impc(m, "ImplicitMPC", "Implicit MPC class");
  impc.def(py::init<const int, const int, const int, const int, const bool, const bool,
           const bool>(), "Constructor", py::arg("num_states"), py::arg("num_inputs"),
           py::arg("len_horizon"), py::arg("num_control_points"),
           py::arg("use_input_cost")=false, py::arg("use_slew_rate")=false,
           py::arg("saturate_states")=false);

  impc.def("getInputTrajectory",
    [](ImplicitMPC& self, Eigen::Ref<Eigen::VectorXd> u_traj)
    {
      self.getInputTrajectory(u_traj);
      return u_traj;
    }, "Get optimal trajectory of inputs from previous solve", py::arg("u_traj"));
  impc.def("getInputTrajectory",
    [](ImplicitMPC& self)
    {
      Eigen::VectorXd u_traj{self.getNumInputs()*self.getHorizonLength()};
      self.getInputTrajectory(u_traj);
      return u_traj;
    }, "Get optimal trajectory of inputs from previous solve");

  impc.def("getPredictedStateTrajectory",
    [](ImplicitMPC& self, Eigen::Ref<Eigen::VectorXd> x_traj)
    {
      self.getPredictedStateTrajectory(x_traj);
      return x_traj;
    }, "Get predicted state trajectory from previous solve", py::arg("x_traj"));
  impc.def("getPredictedStateTrajectory",
    [](ImplicitMPC& self)
    {
      Eigen::VectorXd x_traj{self.getNumStates()*self.getHorizonLength()};
      self.getPredictedStateTrajectory(x_traj);
      return x_traj;
    }, "Get predicted state trajectory from previous solve");

  impc.def("setInputLimits", &ImplicitMPC::setInputLimits, "Set input saturation limits",
    py::arg("u_min"), py::arg("u_max"));
  impc.def("setStateLimits", &ImplicitMPC::setStateLimits, "Set state saturation limits",
    py::arg("x_min"), py::arg("x_max"));
  impc.def("setSlewRate", &ImplicitMPC::setSlewRate, "Set slew rate constraint limits",
    py::arg("u_slew"));
}

} // namespace affine_mpc
