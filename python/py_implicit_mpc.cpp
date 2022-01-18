#include "affine_mpc_py_module.hpp"
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include "affine_mpc/implicit_mpc.hpp"

namespace py = pybind11;

void moduleAddImplicitMPC(py::module& m)
{
  py::class_<ImplicitMPC> impc(m, "ImplicitMPC");
  impc.def(py::init<const int, const int, const int, const int, const bool, const bool,
           const bool>(), "Constructor", py::arg("num_states"), py::arg("num_inputs"),
           py::arg("horizon_length"), py::arg("num_knot_points"),
           py::arg("use_input_cost")=false, py::arg("use_slew_rate")=false,
           py::arg("saturate_states")=false);

  impc.def("calcNextInput",
    [](ImplicitMPC& self, const Ref<const VectorXd>& x0, Ref<VectorXd> u)
    {
      bool success = self.calcNextInput(x0, u);
      return std::make_tuple(u, success);
    }, "Propagate model", py::arg("x0"), py::arg("u"));
  impc.def("calcNextInput",
    [](ImplicitMPC& self, const Ref<const VectorXd>& x0)
    {
      VectorXd u{self.getNumInputs()};
      bool success = self.calcNextInput(x0, u);
      return std::make_tuple(u, success);
    }, "Propagate model", py::arg("x0"));

  impc.def("calcInputTrajectory",
    [](ImplicitMPC& self, const Ref<const VectorXd>& x0, Ref<VectorXd> u_traj)
    {
      bool success = self.calcInputTrajectory(x0, u_traj);
      return std::make_tuple(u_traj, success);
    }, "Calculate optimal trajectory of inputs", py::arg("x0"), py::arg("u_traj"));
  impc.def("calcInputTrajectory",
    [](ImplicitMPC& self, const Ref<const VectorXd>& x0)
    {
      VectorXd u_traj{self.getNumInputs()*self.getNumKnotPoints()};
      bool success = self.calcInputTrajectory(x0, u_traj);
      return std::make_tuple(u_traj, success);
    }, "Propagate model", py::arg("x0"));

  impc.def("initSolver", &ImplicitMPC::initSolver,
    "Initialize OSQP solver after configuring MPC setup",
    py::arg("solver_settings").none(true));

  impc.def("propagateModel",
    [](ImplicitMPC& self, const Ref<const VectorXd>& x0, const Ref<const VectorXd>& u,
       Ref<VectorXd> x_next)
    {
      self.propagateModel(x0, u, x_next);
      return x_next;
    }, "Simulate internal model propagation", py::arg("x0"), py::arg("u"),
    py::arg("x_next"));
  impc.def("propagateModel",
    [](ImplicitMPC& self, const Ref<const VectorXd>& x0, const Ref<const VectorXd>& u)
    {
      VectorXd x_next{self.getNumStates()};
      self.propagateModel(x0, u, x_next);
      return x_next;
    }, "Simulate internal model propagation", py::arg("x0"), py::arg("u"));

  impc.def("setModelDiscrete", &ImplicitMPC::setModelDiscrete,
    "Set internal model directly from discrete model", py::arg("Ad"), py::arg("Bd"),
    py::arg("wd"));
  impc.def("setModelContinuous2Discrete", &ImplicitMPC::setModelContinuous2Discrete,
    "Set internal model from discretized continuous model", py::arg("Ac"), py::arg("Bc"),
    py::arg("wc"), py::arg("dt"), py::arg("tol")=1e-6);

  impc.def("setWeights", &ImplicitMPC::setWeights, "Set state and input weights",
    py::arg("Q_diag"), py::arg("R_diag"));
  impc.def("setStateWeights", &ImplicitMPC::setStateWeights, "Set state weights",
    py::arg("Q_diag"));
  impc.def("setInputWeights", &ImplicitMPC::setInputWeights,
    "Set state and input weights", py::arg("R_diag"));

  impc.def("setDesiredState", &ImplicitMPC::setDesiredState,
    "Set desired state trajectory as a step command", py::arg("x_step"));
  impc.def("setDesiredInput", &ImplicitMPC::setDesiredInput,
    "Set desired input trajectory as a step command", py::arg("u_step"));
  impc.def("setDesiredStateTrajectory", &ImplicitMPC::setDesiredStateTrajectory,
    "Set desired state trajectory", py::arg("x_traj"));
  impc.def("setDesiredInputTrajectory", &ImplicitMPC::setDesiredInputTrajectory,
    "Set desired input trajectory", py::arg("u_traj"));

  impc.def("setInputLimits", &ImplicitMPC::setInputLimits, "Set input saturation limits",
    py::arg("u_min"), py::arg("u_max"));
  impc.def("setStateLimits", &ImplicitMPC::setStateLimits, "Set state saturation limits",
    py::arg("x_min"), py::arg("x_max"));
  impc.def("setSlewRate", &ImplicitMPC::setSlewRate, "Set slew rate constraint limits",
    py::arg("u_slew"));

  impc.def_property_readonly("num_states",
    [](ImplicitMPC& self) { return self.getNumStates(); });
  impc.def_property_readonly("num_inputs",
    [](ImplicitMPC& self) { return self.getNumInputs(); });
  impc.def_property_readonly("horizon_length",
    [](ImplicitMPC& self) { return self.getHorizonLength(); });
  impc.def_property_readonly("num_knot_points",
    [](ImplicitMPC& self) { return self.getNumKnotPoints(); });
}
