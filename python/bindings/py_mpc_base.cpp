#include "affine_mpc_py_module.hpp"

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

#include "affine_mpc/mpc_base.hpp"

namespace affine_mpc {
namespace py = pybind11;

// see Pybind11 docs:
// Classes -> Overriding virtual functions in Python
// Classes -> Binding protected member functions
class PyMPCBase : public MPCBase
{
public:
  using MPCBase::MPCBase;
  void convertToQP(const Eigen::Ref<const Eigen::VectorXd>& x0) override
  {
    PYBIND11_OVERLOAD_PURE(void, MPCBase, convertToQP, x0);
  }
};


void moduleAddMPCBase(py::module& m)
{
  py::class_<MPCBase, PyMPCBase> base(m, "MPCBase", "Not usable in Python!");
  base.def(py::init<const int, const int, const int, const int, const int,
                    const Eigen::Ref<const Eigen::VectorXd>&, const bool,
                    const bool, const bool>(),
           "Constructor", py::arg("num_states"), py::arg("num_inputs"),
           py::arg("len_horizon"), py::arg("num_control_points"),
           py::arg("degree"), py::arg("knots") = Eigen::VectorXd(0),
           py::arg("use_input_cost") = false, py::arg("use_slew_rate") = false,
           py::arg("saturate_states") = false);

  base.def("initializeSolver", &MPCBase::initializeSolver,
           "Initialize OSQP solver after configuring MPC setup",
           py::arg("solver_settings").none(true) = nullptr);

  base.def("solve", &MPCBase::solve, "Solve optimization problem",
           py::arg("x0"));

  base.def(
      "getNextInput",
      [](MPCBase& self, Eigen::Ref<Eigen::VectorXd> u0) {
        self.getNextInput(u0);
        return u0;
      },
      "Get next input to apply, i.e. first input in horizon from previous "
      "solve",
      py::arg("u0"));
  base.def(
      "getNextInput",
      [](MPCBase& self) {
        Eigen::VectorXd u0{self.getNumInputs()};
        self.getNextInput(u0);
        return u0;
      },
      "Get next input to apply, i.e. first input in horizon from previous "
      "solve");

  base.def(
      "getParameterizedInputTrajectory",
      [](MPCBase& self, Eigen::Ref<Eigen::VectorXd> u_traj_ctrl_pts) {
        self.getParameterizedInputTrajectory(u_traj_ctrl_pts);
        return u_traj_ctrl_pts;
      },
      "Get optimal parameterized trajectory of inputs from previous solve",
      py::arg("u_traj_ctrl_pts"));
  base.def(
      "getParameterizedInputTrajectory",
      [](MPCBase& self) {
        Eigen::VectorXd u_traj_ctrl_pts{self.getNumInputs() *
                                        self.getNumControlPoints()};
        self.getParameterizedInputTrajectory(u_traj_ctrl_pts);
        return u_traj_ctrl_pts;
      },
      "Get optimal parameterized trajectory of inputs from previous solve");

  base.def(
      "getInputTrajectory",
      [](MPCBase& self, Eigen::Ref<Eigen::VectorXd> u_traj) {
        self.getInputTrajectory(u_traj);
        return u_traj;
      },
      "Get optimal trajectory of inputs from previous solve",
      py::arg("u_traj"));
  base.def(
      "getInputTrajectory",
      [](MPCBase& self) {
        Eigen::VectorXd u_traj{self.getNumInputs() * self.getHorizonLength()};
        self.getInputTrajectory(u_traj);
        return u_traj;
      },
      "Get optimal trajectory of inputs from previous solve");

  base.def(
      "getPredictedStateTrajectory",
      [](MPCBase& self, Eigen::Ref<Eigen::VectorXd> x_traj) {
        self.getPredictedStateTrajectory(x_traj);
        return x_traj;
      },
      "Get predicted state trajectory from previous solve", py::arg("x_traj"));
  base.def(
      "getPredictedStateTrajectory",
      [](MPCBase& self) {
        Eigen::VectorXd x_traj{self.getNumStates() * self.getHorizonLength()};
        self.getPredictedStateTrajectory(x_traj);
        return x_traj;
      },
      "Get predicted state trajectory from previous solve");

  base.def(
      "propagateModel",
      [](MPCBase& self, const Eigen::Ref<const Eigen::VectorXd>& x,
         const Eigen::Ref<const Eigen::VectorXd>& u,
         Eigen::Ref<Eigen::VectorXd> x_next) {
        self.propagateModel(x, u, x_next);
        return x_next;
      },
      "Simulate internal model propagation step", py::arg("x"), py::arg("u"),
      py::arg("x_next"));
  base.def(
      "propagateModel",
      [](MPCBase& self, const Eigen::Ref<const Eigen::VectorXd>& x,
         const Eigen::Ref<const Eigen::VectorXd>& u) {
        Eigen::VectorXd x_next{self.getNumStates()};
        self.propagateModel(x, u, x_next);
        return x_next;
      },
      "Simulate internal model propagation step", py::arg("x"), py::arg("u"));

  base.def("setModelDiscrete", &MPCBase::setModelDiscrete,
           "Set internal model directly from discrete model", py::arg("Ad"),
           py::arg("Bd"), py::arg("wd"));
  base.def("setModelContinuous2Discrete", &MPCBase::setModelContinuous2Discrete,
           "Set internal model from discretized continuous model",
           py::arg("Ac"), py::arg("Bc"), py::arg("wc"), py::arg("dt"),
           py::arg("tol") = 1e-6);

  base.def("setWeights", &MPCBase::setWeights, "Set state and input weights",
           py::arg("Q_diag"), py::arg("R_diag"));
  base.def("setStateWeights", &MPCBase::setStateWeights, "Set state weights",
           py::arg("Q_diag"));
  base.def("setStateWeightsTerminal", &MPCBase::setStateWeightsTerminal,
           "Set state weights", py::arg("Qf_diag"));
  base.def("setInputWeights", &MPCBase::setInputWeights,
           "Set state and input weights", py::arg("R_diag"));

  base.def("setReferenceState", &MPCBase::setReferenceState,
           "Set desired state trajectory as a step command", py::arg("x_step"));
  base.def("setReferenceInput", &MPCBase::setReferenceInput,
           "Set desired input trajectory as a step command", py::arg("u_step"));
  base.def("setReferenceStateTrajectory", &MPCBase::setReferenceStateTrajectory,
           "Set desired state trajectory", py::arg("x_traj"));
  base.def("setReferenceParameterizedInputTrajectory",
           &MPCBase::setReferenceParameterizedInputTrajectory,
           "Set desired input trajectory", py::arg("u_traj_ctrl_pts"));

  base.def("setInputLimits", &MPCBase::setInputLimits,
           "Set input saturation limits", py::arg("u_min"), py::arg("u_max"));
  base.def("setStateLimits", &MPCBase::setStateLimits,
           "Set state saturation limits", py::arg("x_min"), py::arg("x_max"));
  base.def("setSlewRate", &MPCBase::setSlewRate,
           "Set slew rate constraint limits", py::arg("u_slew"));

  base.def_property_readonly("num_states",
                             [](MPCBase& self) { return self.getNumStates(); });
  base.def_property_readonly("num_inputs",
                             [](MPCBase& self) { return self.getNumInputs(); });
  base.def_property_readonly(
      "len_horizon", [](MPCBase& self) { return self.getHorizonLength(); });
  base.def_property_readonly(
      "num_ctrl_pts", [](MPCBase& self) { return self.getNumControlPoints(); });

  // MPCBase can't work in Python anyways because derived classes resize Eigen
  // variables base.def("_convertToQP", static_cast<void (MPCBase::*)(const
  // Eigen::Ref<const Eigen::VectorXd>&)>(&PyMPCBase::convertToQP),
  //   "PRIVATE - do not call manually! Defines how to convert MPC problem to a
  //   QP problem", py::arg("x0"));
}

} // namespace affine_mpc
