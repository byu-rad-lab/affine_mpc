#include "affine_mpc_py_module.hpp"

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

#include "affine_mpc/mpc_base.hpp"
#include "affine_mpc/options.hpp"
#include "affine_mpc/parameterization.hpp"

namespace affine_mpc_py {
namespace ampc = affine_mpc;
namespace py = pybind11;

// see Pybind11 docs:
// Classes -> Overriding virtual functions in Python
// Classes -> Binding protected member functions
class PyMPCBase : public ampc::MPCBase
{
public:
  PyMPCBase(int state_dim,
            int input_dim,
            const ampc::Parameterization& parameterization,
            const ampc::Options& opts,
            int num_design_vars,
            int num_custom_constraints) :
      MPCBase(state_dim,
              input_dim,
              parameterization,
              opts,
              num_design_vars,
              num_custom_constraints)
  {
    // Allow creation for unit tests. Still abstract and unusable.
  }
  ~PyMPCBase() override = default;

  // define pure virtual funtions for unit testing
  void qpUpdateX0(const Eigen::Ref<const Eigen::VectorXd>& x0) override
  {
    PYBIND11_OVERLOAD_PURE(void, MPCBase, qpUpdateX0, x0);
  }
  bool qpUpdateModel() override { return true; }
  bool qpUpdateReferences() override { return true; }
  bool qpUpdateInputLimits() override { return true; }
  bool qpUpdateStateLimits() override { return true; }
  bool qpUpdateSlewRate() override { return true; }
};

void moduleAddMPCBase(py::module& m)
{
  py::class_<ampc::MPCBase, PyMPCBase> base(
      m, "MPCBase", "Abstract class. Not usable on its own.");
  base.def(py::init<const int, const int, const ampc::Parameterization&,
                    const ampc::Options&, const int, const int>(),
           R"doc(
Constructor for MPCBase.

This is an abstract class and not meant to be used directly, but is still
defined here to allow for testing and potentially to allow for users to create
their own custom MPC classes in Python by inheriting from this class and
implementing the pure virtual functions.

Args:
    state_dim: Dimension of state vector.
    input_dim: Dimension of input vector.
    parameterization: Input trajectory parameterization.
    opts: Optional MPC configuration features to enable.
    num_design_vars: Number of decision variables in the optimization problem.
    num_custom_constraints: Number of custom constraints in the optimization
        problem (beyond those implemented by MPCBase: input/state limits, slew
        rates, and input trajectory saturation).
           )doc",
           py::arg("state_dim"), py::arg("input_dim"), py::arg("param"),
           py::arg("opts"), py::arg("num_design_vars"),
           py::arg("num_custom_constraints"));

  base.def("initializeSolver", &ampc::MPCBase::initializeSolver,
           R"doc(
Initialize OSQP solver after configuring MPC setup. Calling this method after
a successful initialization will simply return True without doing anything.

Prior to calling this function, you must have set the model and input limits.
You must also provide parameters for all enabled options before calling this
function. For example, if state saturation is enabled then `setStateLimits()`
must be called beforehand.

Args:
    solver_settings: OSQP solver settings. Defaults to recommended settings.

Returns:
    success: True if initialization succeeds, false otherwise.
           )doc",
           py::arg("solver_settings") =
               ampc::OSQPSolver::getRecommendedSettings());

  base.def("solve", &ampc::MPCBase::solve,
           R"doc(
Solve optimization problem for the given initial state.

Must have previously called initializeSolver() prior to calling this. Call
getNextInput(), getParameterizedInputTrajectory(), getInputTrajectory(), or
getPredictedStateTrajectory() after calling this function to access the results.

Args:
    x0: Initial (current) state vector.

Returns:
    solve_status: Result indication. Generally expected to be `Success` unless
        the solver has not been initialized, then `NotInitialized`. Verify your
        problem setup and consult OSQP documentation for any other value.
           )doc",
           py::arg("x0"));

  base.def(
      "getNextInput",
      [](ampc::MPCBase& self, Eigen::Ref<Eigen::VectorXd> u0) {
        self.getNextInput(u0);
        return u0;
      },
      R"doc(
Get the next input to apply (initial input from optimized trajectory) from the
previous solve.

Args:
    u0: Result will be stored here (equivalent to return value).

Returns:
    u0: Initial input from optimized trajectory (next to apply).
      )doc",
      py::arg("u0"));
  base.def(
      "getNextInput",
      [](ampc::MPCBase& self) {
        Eigen::VectorXd u0{self.getInputDim()};
        self.getNextInput(u0);
        return u0;
      },
      R"doc(
Get the next input to apply (initial input from optimized trajectory) from the
previous solve.

returns:
    u0: Initial input from optimized trajectory (next to apply).
      )doc");

  base.def(
      "getParameterizedInputTrajectory",
      [](ampc::MPCBase& self, Eigen::Ref<Eigen::VectorXd> u_traj_ctrl_pts) {
        self.getParameterizedInputTrajectory(u_traj_ctrl_pts);
        return u_traj_ctrl_pts;
      },
      R"doc(
Get the parameterized input trajectory (control points) from the previous solve.

Args:
    u_traj_ctrl_pts (vector): Result will be stored here (equivalent to return value).

returns:
    u_traj_ctrl_pts (vector): The parameterized input trajectory.
      )doc",
      py::arg("u_traj_ctrl_pts"));
  base.def(
      "getParameterizedInputTrajectory",
      [](ampc::MPCBase& self) {
        Eigen::VectorXd u_traj_ctrl_pts{self.getInputDim()
                                        * self.getNumControlPoints()};
        self.getParameterizedInputTrajectory(u_traj_ctrl_pts);
        return u_traj_ctrl_pts;
      },
      R"doc(
Get the parameterized input trajectory (control points) from the previous solve.

returns:
    u_traj_ctrl_pts (vector): The parameterized input trajectory.
      )doc");

  base.def(
      "getInputTrajectory",
      [](ampc::MPCBase& self, Eigen::Ref<Eigen::VectorXd> u_traj) {
        self.getInputTrajectory(u_traj);
        return u_traj;
      },
      R"doc(
Get the full input trajectory from the previous solve.

Args:
    u_traj (vector): Result will be stored here (equivalent to return value).

returns:
    u_traj (vector): The input trajectory.
      )doc",
      py::arg("u_traj"));
  base.def(
      "getInputTrajectory",
      [](ampc::MPCBase& self) {
        Eigen::VectorXd u_traj{self.getInputDim() * self.getHorizonSteps()};
        self.getInputTrajectory(u_traj);
        return u_traj;
      },
      R"doc(
Get the full input trajectory from the previous solve.

returns:
    u_traj (vector): The input trajectory.
      )doc");

  base.def(
      "getPredictedStateTrajectory",
      [](ampc::MPCBase& self, Eigen::Ref<Eigen::VectorXd> x_traj) {
        self.getPredictedStateTrajectory(x_traj);
        return x_traj;
      },
      R"doc(
Get the predicted state trajectory from the previous solve.

Args:
    x_traj (vector): Result will be stored here (equivalent to return value).

returns:
    x_traj (vector): The predicted state trajectory.
      )doc",
      py::arg("x_traj"));
  base.def(
      "getPredictedStateTrajectory",
      [](ampc::MPCBase& self) {
        Eigen::VectorXd x_traj{self.getStateDim() * self.getHorizonSteps()};
        self.getPredictedStateTrajectory(x_traj);
        return x_traj;
      },
      R"doc(
Get the predicted state trajectory from the previous solve.

returns:
    x_traj (vector): The predicted state trajectory.
      )doc");

  base.def(
      "propagateModel",
      [](ampc::MPCBase& self, const Eigen::Ref<const Eigen::VectorXd>& x,
         const Eigen::Ref<const Eigen::VectorXd>& u,
         Eigen::Ref<Eigen::VectorXd> x_next) {
        self.propagateModel(x, u, x_next);
        return x_next;
      },
      R"doc(
Propagate the internal discrete-time model for one step.

Model must be set prior to calling this method.

Args:
    x: Current state vector.
    u: Current input vector.
    x_next (out): Resulting next state vector (equivalent to return value).

Returns:
    x_next : Resulting next state vector.
      )doc",
      py::arg("x"), py::arg("u"), py::arg("x_next"));
  base.def(
      "propagateModel",
      [](ampc::MPCBase& self, const Eigen::Ref<const Eigen::VectorXd>& x,
         const Eigen::Ref<const Eigen::VectorXd>& u) {
        Eigen::VectorXd x_next{self.getStateDim()};
        self.propagateModel(x, u, x_next);
        return x_next;
      },
      R"doc(
Propagate the internal discrete-time model for one step.

Model must be set prior to calling this method.

Args:
    x: Current state vector.
    u: Current input vector.

Returns:
    x_next : Resulting next state vector.
      )doc",
      py::arg("x"), py::arg("u"));

  base.def("setModelDiscrete", &ampc::MPCBase::setModelDiscrete,
           R"doc(
Set the internal discrete-time model directly from a discrete model. The model
being `x_next = Ax + Bu + w`.

Args:
    Ad: Discrete-time state matrix A.
    Bd: Discrete-time input matrix B.
    wd: Discrete-time affine/bias vector w.

Returns:
    success: True if the internal QP was updated properly. Unlikely to be False.
           )doc",
           py::arg("Ad"), py::arg("Bd"), py::arg("wd"));
  base.def("setModelContinuous2Discrete",
           &ampc::MPCBase::setModelContinuous2Discrete,
           R"doc(
Set the internal discrete-time model from a continuous-time model, which will be
discretized. The model being `x_next = Ax + Bu + w`.

The discretization assumes the input u is constant over the time step (true for
discrete controllers like MPC) and uses a matrix exponential, which is an exact
discretization (theoretically). The matrix exponential involves a Taylor series
expansion $\sum_{i=0}^\inf (A*dt)^i / i!$. Thus the scalar term is `dt^i / i!`.
The infinite summation is stopped when this term becomes smaller than `tol`.

Args:
    Ac: Continuous-time state matrix A.
    Bc: Continuous-time input matrix B.
    wc: Continuous-time affine/bias vector w.
    dt (seconds): Discretization time step. Usually should be much smaller than
        1s for numeric stability reasons. This usually matches the control rate,
        and the input is held constant for this duration.
    tol: Tolerance for the matrix exponential. Taylor series expansion stops
        once scalar multiplier becomes smaller than this value.

Returns:
    success: True if the internal QP was updated properly. Unlikely to be False.
           )doc",
           py::arg("Ac"), py::arg("Bc"), py::arg("wc"), py::arg("dt"),
           py::arg("tol") = 1e-6);

  base.def(
      "setWeights",
      [](ampc::MPCBase& self, const Eigen::Ref<const Eigen::VectorXd>& Q_diag,
         const Eigen::Ref<const Eigen::VectorXd>& R_diag) {
        self.setWeights(Q_diag, R_diag);
      },
      R"doc(
Set state and input weights for the cost function.

This method can only be called if `use_input_cost` is enabled.

Args:
    Q_diag: Vector for diagonal of state weight matrix (non-negative).
    R_diag: Vector for diagonal of input weight matrix (non-negative).
      )doc",
      py::arg("Q_diag"), py::arg("R_diag"));
  base.def(
      "setWeights",
      [](ampc::MPCBase& self, const Eigen::Ref<const Eigen::VectorXd>& Q_diag,
         const Eigen::Ref<const Eigen::VectorXd>& Qf_diag,
         const Eigen::Ref<const Eigen::VectorXd>& R_diag) {
        self.setWeights(Q_diag, R_diag);
      },
      R"doc(
Set state, terminal state, and input weights for the cost function.

This method can only be called if `use_input_cost` is enabled.

Args:
    Q_diag: Vector for diagonal of state weight matrix (non-negative).
    Qf_diag: Vector for diagonal of terminal state weight matrix (non-negative).
    R_diag: Vector for diagonal of input weight matrix (non-negative).
      )doc",
      py::arg("Q_diag"), py::arg("Qf_diag"), py::arg("R_diag"));

  base.def(
      "setStateWeights",
      [](ampc::MPCBase& self, const Eigen::Ref<const Eigen::VectorXd>& Q_diag) {
        self.setStateWeights(Q_diag);
      },
      R"doc(
Set only the state weights for the cost function.

Args:
    Q_diag: Vector for diagonal of state weight matrix (non-negative).
      )doc",
      py::arg("Q_diag"));
  base.def(
      "setStateWeights",
      [](ampc::MPCBase& self, const Eigen::Ref<const Eigen::VectorXd>& Q_diag,
         const Eigen::Ref<const Eigen::VectorXd>& Qf_diag) {
        self.setStateWeights(Q_diag, Qf_diag);
      },
      R"doc(
Set only the state and terminal state weights for the cost function.

Args:
    Q_diag: Vector for diagonal of state weight matrix (non-negative).
    Qf_diag: Vector for diagonal of terminal state weight matrix (non-negative).
      )doc",
      py::arg("Q_diag"), py::arg("Qf_diag"));

  base.def("setInputWeights", &ampc::MPCBase::setInputWeights,
           R"doc(
Set only the input weights for the cost function.

This method can only be called if `use_input_cost` is enabled.

Args:
    R_diag: Vector for diagonal of input weight matrix (non-negative).
           )doc",
           py::arg("R_diag"));

  base.def("setReferenceState", &ampc::MPCBase::setReferenceState,
           R"doc(
Set reference state trajectory as a step command.

Args:
    x_step: Reference state vector to use for entire trajectory.

Returns:
    success: True if the internal QP was updated properly. Unlikely to be False.
           )doc",
           py::arg("x_step"));

  base.def("setReferenceStateTrajectory",
           &ampc::MPCBase::setReferenceStateTrajectory,
           R"doc(
Set reference state trajectory.

Args:
    x_traj (vector): Reference state trajectory as a vector of stacked states.
        Should have length of state_dim * horizon_steps. If you have a matrix
        where each row is the state at index k in the horizon, then you can pass
        in `x_traj.ravel()` to convert it to a vector.

Returns:
    success: True if the internal QP was updated properly. Unlikely to be False.
           )doc",
           py::arg("x_traj"));

  base.def("setReferenceInput", &ampc::MPCBase::setReferenceInput,
           R"doc(
Set reference input trajectory as a step command (reference for all control
points set to this value).

This method can only be called if `use_input_cost` is enabled.

Args:
    u_step: Reference input vector to use for entire trajectory.

Returns:
    success: True if the internal QP was updated properly. Unlikely to be False.
           )doc",
           py::arg("u_step"));

  base.def("setReferenceParameterizedInputTrajectory",
           &ampc::MPCBase::setReferenceParameterizedInputTrajectory,
           R"doc(
Set reference control points for the input trajectory.

This method can only be called if `use_input_cost` is enabled.

Args:
    u_traj_ctrl_pts (vector): Reference control points as a vector of stacked
        inputs. Should have length of input_dim * num_control_points. If you
        have a matrix where each row is a control point, then you can pass
        in `u_traj_ctrl_pts.ravel()` to convert it to a vector.

Returns:
    success: True if the internal QP was updated properly. Unlikely to be False.
           )doc",
           py::arg("u_traj_ctrl_pts"));

  base.def("setInputLimits", &ampc::MPCBase::setInputLimits,
           R"doc(
Set input saturation limits.

This function must be called prior to `initializeSolver()`, but can also be
called after if limits change between solves.

Args:
    u_min: Lower bound on input vector.
    u_max: Upper bound on input vector.

Returns:
    success: True if the internal QP was updated properly. Unlikely to be False.
           )doc",
           py::arg("u_min"), py::arg("u_max"));
  base.def("setStateLimits", &ampc::MPCBase::setStateLimits,
           R"doc(
Set state saturation limits.

This function can only be called if `saturate_states` is enabled. Must be
called prior to `initializeSolver()`, but can also be called after if limits
change between solves.

Args:
    x_min: Lower bound on state vector.
    x_max: Upper bound on state vector.

Returns:
    success: True if the internal QP was updated properly. Unlikely to be False.
           )doc",
           py::arg("x_min"), py::arg("x_max"));
  base.def("setSlewRate", &ampc::MPCBase::setSlewRate,
           R"doc(
Set slew-rate constraint limits for control points. Control points can vary by
up to this magnitude from one index to the next.

This function can only be called if `slew_control_points` is enabled. Must be
called prior to `initializeSolver()`, but can also be called after if limits
change between solves.

Args:
    u_slew: Slew-rate vector. All values must be positive.

Returns:
    success: True if the internal QP was updated properly. Unlikely to be False.
           )doc",
           py::arg("u_slew"));

  base.def("setSlewRateInitial", &ampc::MPCBase::setSlewRateInitial,
           R"doc(
Set initial slew-rate constraint limits for control points. Initial input can
vary by up to this magnitude from the previous input. See `setPreviousInput()`.

This function can only be called if `slew_initial_input` is enabled. Must be
called prior to `initializeSolver()`, but can also be called after if limits
change between solves.

Args:
    u0_slew: Initial slew-rate vector. All values must be positive.

Returns:
    success: True if the internal QP was updated properly. Unlikely to be False.
           )doc",
           py::arg("u0_slew"));

  base.def("setPreviousInput", &ampc::MPCBase::setPreviousInput,
           R"doc(
Set the previous input, which is only used with the `slew_initial_input`
constraint option. Defaults to zeros, but should be set prior to first solve if
a different value is desired.

This value is automatically updated after each solve, so you really only need to
call this function once before the first solve, or potentially if a solve fails
and you want to set a custom strategy for handling it.

This function can only be called if `slew_initial_input` is enabled. Must be
called prior to `initializeSolver()`, but can also be called after if limits
change between solves.

Args:
    u_prev: Previous input vector.

Returns:
    success: True if the internal QP was updated properly. Unlikely to be False.
           )doc",
           py::arg("u_prev"));

  base.def_property_readonly(
      "state_dim", [](ampc::MPCBase& self) { return self.getStateDim(); });
  base.def_property_readonly(
      "input_dim", [](ampc::MPCBase& self) { return self.getInputDim(); });
  base.def_property_readonly("horizon_steps", [](ampc::MPCBase& self) {
    return self.getHorizonSteps();
  });
  base.def_property_readonly("num_control_points", [](ampc::MPCBase& self) {
    return self.getNumControlPoints();
  });

  // MPCBase can't work in Python anyways because derived classes resize Eigen
  // variables base.def("_qpUpdateX0", static_cast<void (MPCBase::*)(const
  // Eigen::Ref<const Eigen::VectorXd>&)>(&PyMPCBase::qpUpdateX0),
  //   "PRIVATE - do not call manually! Defines how to convert MPC problem to a
  //   QP problem", py::arg("x0"));
}

} // namespace affine_mpc_py
