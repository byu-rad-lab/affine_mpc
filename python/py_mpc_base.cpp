#include "affine_mpc_py_module.hpp"
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include "affine_mpc/mpc_base.hpp"

namespace py = pybind11;

// see Pybind11 docs:
// Classes -> Overriding virtual functions in Python
// Classes -> Binding protected member functions
class PyMPCBase : public MPCBase
{
public:
  using MPCBase::MPCBase;
  void convertToQP(const Ref<const VectorXd>& x0) override
  {
    PYBIND11_OVERLOAD_PURE(void, MPCBase, convertToQP, x0);
  }
};


void moduleAddMPCBase(py::module& m)
{
  py::class_<MPCBase, PyMPCBase> base(m, "MPCBase", "Not usable in Python!");
  base.def(py::init<const int, const int, const int, const int, const bool, const bool,
           const bool>(), "Constructor", py::arg("num_states"), py::arg("num_inputs"),
           py::arg("horizon_length"), py::arg("num_knot_points"),
           py::arg("use_input_cost")=false, py::arg("use_slew_rate")=false,
           py::arg("saturate_states")=false);

  base.def("calcNextInput",
    [](MPCBase& self, const Ref<const VectorXd>& x0, Ref<VectorXd> u)
    {
      bool success = self.calcNextInput(x0, u);
      return std::make_tuple(u, success);
    }, "Solve optimization and return only the next input to apply",
    py::arg("x0"), py::arg("u"));
  base.def("calcNextInput",
    [](MPCBase& self, const Ref<const VectorXd>& x0)
    {
      VectorXd u{self.getNumInputs()};
      bool success = self.calcNextInput(x0, u);
      return std::make_tuple(u, success);
    }, "Solve optimization and return only the next input to apply", py::arg("x0"));

  base.def("calcInputTrajectory",
    [](MPCBase& self, const Ref<const VectorXd>& x0, Ref<VectorXd> u_traj)
    {
      bool success = self.calcInputTrajectory(x0, u_traj);
      return std::make_tuple(u_traj, success);
    }, "Calculate optimal parameterized trajectory of inputs",
    py::arg("x0"), py::arg("u_traj"));
  base.def("calcInputTrajectory",
    [](MPCBase& self, const Ref<const VectorXd>& x0)
    {
      VectorXd u_traj{self.getNumInputs()*self.getNumKnotPoints()};
      bool success = self.calcInputTrajectory(x0, u_traj);
      return std::make_tuple(u_traj, success);
    }, "Calculate optimal parameterized trajectory of inputs", py::arg("x0"));

  base.def("unparameterizeSolution",
    [](MPCBase& self, const Ref<const VectorXd>& sol, Ref<VectorXd> u_traj)
    {
      self.unparameterizeSolution(sol, u_traj);
      return u_traj;
    }, "Calculate optimal parameterized trajectory of inputs",
    py::arg("sol"), py::arg("u_traj"));
  base.def("unparameterizeSolution",
    [](MPCBase& self, const Ref<const VectorXd>& sol)
    {
      VectorXd u_traj{self.getNumInputs()*self.getHorizonLength()};
      self.unparameterizeSolution(sol, u_traj);
      return u_traj;
    }, "Calculate optimal parameterized trajectory of inputs", py::arg("sol"));

  base.def("getPredictedStateTrajectory",
    [](MPCBase& self, Ref<VectorXd> x_traj)
    {
      self.getPredictedStateTrajectory(x_traj);
      return x_traj;
    }, "Get predicted state trajectory from previous solve", py::arg("x_traj"));
  base.def("getPredictedStateTrajectory",
    [](MPCBase& self)
    {
      VectorXd x_traj{self.getNumStates()*self.getHorizonLength()};
      self.getPredictedStateTrajectory(x_traj);
      return x_traj;
    }, "Calculate optimal parameterized trajectory of inputs");

  base.def("initSolver", &MPCBase::initSolver,
    "Initialize OSQP solver after configuring MPC setup",
    py::arg("solver_settings").none(true)=nullptr);

  base.def("propagateModel",
    [](MPCBase& self, const Ref<const VectorXd>& x0, const Ref<const VectorXd>& u,
       Ref<VectorXd> x_next)
    {
      self.propagateModel(x0, u, x_next);
      return x_next;
    }, "Simulate internal model propagation", py::arg("x0"), py::arg("u"),
    py::arg("x_next"));
  base.def("propagateModel",
    [](MPCBase& self, const Ref<const VectorXd>& x0, const Ref<const VectorXd>& u)
    {
      VectorXd x_next{self.getNumStates()};
      self.propagateModel(x0, u, x_next);
      return x_next;
    }, "Simulate internal model propagation", py::arg("x0"), py::arg("u"));

  base.def("setModelDiscrete", &MPCBase::setModelDiscrete,
    "Set internal model directly from discrete model", py::arg("Ad"), py::arg("Bd"),
    py::arg("wd"));
  base.def("setModelContinuous2Discrete", &MPCBase::setModelContinuous2Discrete,
    "Set internal model from discretized continuous model", py::arg("Ac"), py::arg("Bc"),
    py::arg("wc"), py::arg("dt"), py::arg("tol")=1e-6);

  base.def("setWeights", &MPCBase::setWeights, "Set state and input weights",
    py::arg("Q_diag"), py::arg("R_diag"));
  base.def("setStateWeights", &MPCBase::setStateWeights, "Set state weights",
    py::arg("Q_diag"));
  base.def("setStateWeightsTerminal", &MPCBase::setStateWeightsTerminal,
    "Set state weights", py::arg("Qf_diag"));
  base.def("setInputWeights", &MPCBase::setInputWeights,
    "Set state and input weights", py::arg("R_diag"));

  base.def("setDesiredState", &MPCBase::setDesiredState,
    "Set desired state trajectory as a step command", py::arg("x_step"));
  base.def("setDesiredInput", &MPCBase::setDesiredInput,
    "Set desired input trajectory as a step command", py::arg("u_step"));
  base.def("setDesiredStateTrajectory", &MPCBase::setDesiredStateTrajectory,
    "Set desired state trajectory", py::arg("x_traj"));
  base.def("setDesiredInputTrajectory", &MPCBase::setDesiredInputTrajectory,
    "Set desired input trajectory", py::arg("u_traj"));

  base.def("setInputLimits", &MPCBase::setInputLimits, "Set input saturation limits",
    py::arg("u_min"), py::arg("u_max"));
  base.def("setStateLimits", &MPCBase::setStateLimits, "Set state saturation limits",
    py::arg("x_min"), py::arg("x_max"));
  base.def("setSlewRate", &MPCBase::setSlewRate, "Set slew rate constraint limits",
    py::arg("u_slew"));

  base.def_property_readonly("num_states",
    [](MPCBase& self) { return self.getNumStates(); });
  base.def_property_readonly("num_inputs",
    [](MPCBase& self) { return self.getNumInputs(); });
  base.def_property_readonly("horizon_length",
    [](MPCBase& self) { return self.getHorizonLength(); });
  base.def_property_readonly("num_knot_points",
    [](MPCBase& self) { return self.getNumKnotPoints(); });

// MPCBase can't work in Python anyways because derived classes resize Eigen variables
  // base.def("_convertToQP", static_cast<void (MPCBase::*)(const Ref<const VectorXd>&)>(&PyMPCBase::convertToQP),
  //   "PRIVATE - do not call manually! Defines how to convert MPC problem to a QP problem",
  //   py::arg("x0"));
}
