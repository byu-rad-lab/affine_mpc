#include "affine_mpc_py_module.hpp"

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <sstream>
#include <stdexcept>
#include <string>

#include "affine_mpc/mpc_base.hpp"
#include "affine_mpc/options.hpp"
#include "affine_mpc/parameterization.hpp"

namespace affine_mpc_py {
namespace ampc = affine_mpc;
namespace py = pybind11;

// MPCBase's constructor is protected; this subclass makes it constructible
// for unit tests of the shared methods (it uses the condensed formulation)
class PyMPCBase : public ampc::MPCBase
{
public:
  PyMPCBase(int state_dim,
            int input_dim,
            const ampc::Parameterization& parameterization,
            const ampc::Options& opts) :
      MPCBase(state_dim,
              input_dim,
              parameterization,
              opts,
              ampc::detail::FormulationKind::Condensed)
  {}
};

// The C++ solution getters do not check initialization (hot path), but reading
// the solution before initializeSolver() dereferences a null pointer, so guard
// them here to raise a Python exception instead of crashing the interpreter.
static void requireSolverInitialized(const ampc::MPCBase& self,
                                     const char* func_name)
{
  if (!self.isSolverInitialized())
    throw std::logic_error(std::string{"[MPCBase::"} + func_name
                           + "] Solver must be initialized first.");
}

void moduleAddMPCBase(py::module& m)
{
  py::class_<ampc::MPCBase, PyMPCBase> base(
      m, "MPCBase", "Base class. Use CondensedMPC or SparseMPC.");
  base.def(py::init<const int, const int, const ampc::Parameterization&,
                    const ampc::Options&>(),
           R"doc(
Constructor for MPCBase.

This is a base class and not meant to be used directly (use CondensedMPC or
SparseMPC). It is constructible for testing and so derived classes share its
methods. Subclassing it in Python to define a custom formulation is not
supported.

Args:
    state_dim: Dimension of state vector.
    input_dim: Dimension of input vector.
    parameterization: Input trajectory parameterization.
    opts: Optional MPC configuration features to enable.
           )doc",
           py::arg("state_dim"), py::arg("input_dim"), py::arg("param"),
           py::arg("opts"));

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
getNextInput(), getInputControlPoints(), getInputTrajectory(), or
getPredictedStateTrajectory() after calling this function to access the results.

The setters only store their values, so solve() first applies every change since
the last solve() or prepare() to the QP (rebuilding QP matrices and refactoring
for model or weight changes), then the terms that depend on x0, then runs the
solver.

Args:
    x0: Initial (current) state vector.

Returns:
    solve_status: Result indication. Generally expected to be `Success` unless
        the solver has not been initialized, then `NotInitialized`.
        `UpdateFailed` means OSQP rejected a data update (often from
        non-finite model or weight values), so the QP was not solved and the
        getters still return the previous solution. The update is retried at
        the next solve, which usually recovers once the values are valid again
        (infinite weights can leave OSQP unable to recover). Verify your
        problem setup and consult OSQP documentation for any other value.

The GIL is released while solving, so other Python threads can run (e.g.
solving several independent MPC objects in parallel threads). Do not modify x0
or this MPC object from another thread during the call.

Raises:
    ValueError: If x0 does not have size state_dim.
           )doc",
           py::arg("x0"), py::call_guard<py::gil_scoped_release>());

  base.def("prepare", &ampc::MPCBase::prepare,
           R"doc(
Optional: apply every pending change that does not depend on the initial state,
so the next solve() only does the x0-dependent work and the solver iterations.

Setters only store their values; the QP work happens in solve(). For changes of
the model matrices or the weights, that work (rebuilding QP matrices and
refactoring) is most of the cost of a solve. Calling prepare() after these
setters and before the next state measurement (e.g. right after applying the
previous input) moves that work out of the time between measurement and input.
Calling it is never required: solve() applies anything still pending. It saves
little when only the model offset, references, limits, or the previous input
changed (they only update QP vectors), and has nothing to do when nothing
changed (e.g. an LTI problem with fixed weights).

Its duration is added to the `prepare_time` that the next solve() reports in
getSolveInfo(). The GIL is released during the call.

Returns:
    status: `Success` if the pending changes were applied, `NotInitialized`
        before initializeSolver(), or `UpdateFailed` if OSQP rejected a data
        update (the update stays pending, so the next solve() retries it and
        reports `UpdateFailed` again while the data is invalid).
           )doc",
           py::call_guard<py::gil_scoped_release>());

  base.def(
      "getNextInput",
      [](ampc::MPCBase& self, Eigen::Ref<Eigen::VectorXd> u0) {
        requireSolverInitialized(self, "getNextInput");
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

Raises:
    RuntimeError: If the solver has not been initialized.
    ValueError: If u0 has the wrong size.
      )doc",
      py::arg("u0"));
  base.def(
      "getNextInput",
      [](ampc::MPCBase& self) {
        requireSolverInitialized(self, "getNextInput");
        Eigen::VectorXd u0{self.getInputDim()};
        self.getNextInput(u0);
        return u0;
      },
      R"doc(
Get the next input to apply (initial input from optimized trajectory) from the
previous solve.

returns:
    u0: Initial input from optimized trajectory (next to apply).

Raises:
    RuntimeError: If the solver has not been initialized.
      )doc");

  base.def(
      "getInputControlPoints",
      [](ampc::MPCBase& self, Eigen::Ref<Eigen::VectorXd> control_points) {
        requireSolverInitialized(self, "getInputControlPoints");
        self.getInputControlPoints(control_points);
        return control_points;
      },
      R"doc(
Get the control points that parameterize the input trajectory from the previous
solve.

Args:
    control_points (vector): Result will be stored here (equivalent to return
        value).

Returns:
    control_points (vector): The stacked control points.

Raises:
    RuntimeError: If the solver has not been initialized.
    ValueError: If control_points has the wrong size.
      )doc",
      py::arg("control_points"));
  base.def(
      "getInputControlPoints",
      [](ampc::MPCBase& self) {
        requireSolverInitialized(self, "getInputControlPoints");
        Eigen::VectorXd control_points{self.getInputDim()
                                       * self.getNumControlPoints()};
        self.getInputControlPoints(control_points);
        return control_points;
      },
      R"doc(
Get the control points that parameterize the input trajectory from the previous
solve.

Returns:
    control_points (vector): The stacked control points.

Raises:
    RuntimeError: If the solver has not been initialized.
      )doc");

  base.def(
      "getInputTrajectory",
      [](ampc::MPCBase& self, Eigen::Ref<Eigen::VectorXd> u_traj) {
        requireSolverInitialized(self, "getInputTrajectory");
        self.getInputTrajectory(u_traj);
        return u_traj;
      },
      R"doc(
Get the full input trajectory from the previous solve.

Args:
    u_traj (vector): Result will be stored here (equivalent to return value).

returns:
    u_traj (vector): The input trajectory.

Raises:
    RuntimeError: If the solver has not been initialized.
    ValueError: If u_traj has the wrong size.
      )doc",
      py::arg("u_traj"));
  base.def(
      "getInputTrajectory",
      [](ampc::MPCBase& self) {
        requireSolverInitialized(self, "getInputTrajectory");
        Eigen::VectorXd u_traj{self.getInputDim() * self.getHorizonSteps()};
        self.getInputTrajectory(u_traj);
        return u_traj;
      },
      R"doc(
Get the full input trajectory from the previous solve.

returns:
    u_traj (vector): The input trajectory.

Raises:
    RuntimeError: If the solver has not been initialized.
      )doc");

  base.def(
      "getPredictedStateTrajectory",
      [](ampc::MPCBase& self, Eigen::Ref<Eigen::VectorXd> x_traj) {
        requireSolverInitialized(self, "getPredictedStateTrajectory");
        self.getPredictedStateTrajectory(x_traj);
        return x_traj;
      },
      R"doc(
Get the predicted state trajectory from the previous solve.

Args:
    x_traj (vector): Result will be stored here (equivalent to return value).

returns:
    x_traj (vector): The predicted state trajectory.

Raises:
    RuntimeError: If the solver has not been initialized.
    ValueError: If x_traj has the wrong size.
      )doc",
      py::arg("x_traj"));
  base.def(
      "getPredictedStateTrajectory",
      [](ampc::MPCBase& self) {
        requireSolverInitialized(self, "getPredictedStateTrajectory");
        Eigen::VectorXd x_traj{self.getStateDim() * self.getHorizonSteps()};
        self.getPredictedStateTrajectory(x_traj);
        return x_traj;
      },
      R"doc(
Get the predicted state trajectory from the previous solve.

returns:
    x_traj (vector): The predicted state trajectory.

Raises:
    RuntimeError: If the solver has not been initialized.
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
    wd: Discrete-time model offset w.

Note:
    The QP is updated in the next prepare() or solve(), which report update
    failures.
           )doc",
           py::arg("Ad"), py::arg("Bd"), py::arg("wd"));
  base.def("setModelContinuous2Discrete",
           &ampc::MPCBase::setModelContinuous2Discrete,
           R"doc(
Set the internal discrete-time model from a continuous-time model, which will be
discretized. The model being `x_next = Ax + Bu + w`.

The discretization assumes the input u is constant over the time step (true for
discrete controllers like MPC) and uses a matrix exponential, which is an exact
discretization (theoretically).

The discretization runs inside this setter, so its cost is not part of solve()
or prepare() (time it as part of your control loop). The QP matrices are rebuilt
in the next prepare() or solve(). When only the offset changes,
setModelOffsetContinuous2Discrete() is much cheaper.

The matrix exponential is computed with scaling and squaring: the step is halved
`s` times until `||A*h||_1 <= 0.5` (with `h = dt / 2^s`), a Taylor series is
evaluated at `h`, and the result is squared `s` times to recover `dt`. When
`||A*dt||_1 <= 0.5` no squaring is needed. The Taylor series stops once the bound
on the next term, `||A*h||_1^i / i!`, is smaller than `tol`.

Args:
    Ac: Continuous-time state matrix A.
    Bc: Continuous-time input matrix B.
    wc: Continuous-time model offset w.
    dt (seconds): Discretization time step (positive). This usually matches the
        control rate, and the input is held constant for this duration.
    tol: Truncation tolerance for the Taylor series (positive). The actual error
        in the discrete model is typically two to three orders of magnitude
        smaller than `tol`. The default gives results close to machine
        precision. Larger values (e.g., 1e-6) save a few matrix products, which
        can help when the affine approximation is updated every time step.

Note:
    The QP is updated in the next prepare() or solve(), which report update
    failures.
           )doc",
           py::arg("Ac"), py::arg("Bc"), py::arg("wc"), py::arg("dt"),
           py::arg("tol") = 1e-10);

  base.def("setModelOffsetDiscrete", &ampc::MPCBase::setModelOffsetDiscrete,
           R"doc(
Set only the model offset wd of the discrete-time model x_next = Ad x + Bd u +
wd, keeping Ad and Bd.

Much cheaper than setModelDiscrete(): the offset only enters vectors of the QP,
so the next solve() does not rebuild the QP matrices and the solver does not
refactor. Use it when only the offset changes between solves, e.g. a disturbance
estimate for offset-free tracking or a measured exogenous input.

Args:
    wd: Discrete-time model offset.

Raises:
    ValueError: If wd does not have size state_dim.
    RuntimeError: If the model has not been set.
           )doc",
           py::arg("wd"));

  base.def("setModelOffsetContinuous2Discrete",
           &ampc::MPCBase::setModelOffsetContinuous2Discrete,
           R"doc(
Set only the model offset from a continuous-time offset wc, discretized with the
state matrix and time step of the last setModelContinuous2Discrete() call
(wd = G wc, see Discretizer).

Costs one matrix-vector product (no matrix exponential), and like
setModelOffsetDiscrete() does not cause a QP matrix rebuild.

Args:
    wc: Continuous-time model offset.

Raises:
    ValueError: If wc does not have size state_dim.
    RuntimeError: If the model was not last set with
        setModelContinuous2Discrete().
           )doc",
           py::arg("wc"));

  base.def("setWeights",
           py::overload_cast<const Eigen::Ref<const Eigen::VectorXd>&,
                             const Eigen::Ref<const Eigen::VectorXd>&>(
               &ampc::MPCBase::setWeights),
           R"doc(
Set state and input weights for the cost function.

This method can only be called if `use_input_cost` is enabled.

Args:
    Q_diag: Vector for diagonal of state weight matrix (non-negative).
    R_diag: Vector for diagonal of input weight matrix (non-negative).
      )doc",
           py::arg("Q_diag"), py::arg("R_diag"));
  base.def("setWeights",
           py::overload_cast<const Eigen::Ref<const Eigen::VectorXd>&,
                             const Eigen::Ref<const Eigen::VectorXd>&,
                             const Eigen::Ref<const Eigen::VectorXd>&>(
               &ampc::MPCBase::setWeights),
           R"doc(
Set state, terminal state, and input weights for the cost function.

This method can only be called if `use_input_cost` is enabled.

Args:
    Q_diag: Vector for diagonal of state weight matrix (non-negative).
    Qf_diag: Vector for diagonal of terminal state weight matrix (non-negative).
    R_diag: Vector for diagonal of input weight matrix (non-negative).
      )doc",
           py::arg("Q_diag"), py::arg("Qf_diag"), py::arg("R_diag"));

  base.def("setStateWeights",
           py::overload_cast<const Eigen::Ref<const Eigen::VectorXd>&>(
               &ampc::MPCBase::setStateWeights),
           R"doc(
Set only the state weights for the cost function.

The terminal state weights are also set to `Q_diag`.

Args:
    Q_diag: Vector for diagonal of state weight matrix (non-negative).
      )doc",
           py::arg("Q_diag"));
  base.def("setStateWeights",
           py::overload_cast<const Eigen::Ref<const Eigen::VectorXd>&,
                             const Eigen::Ref<const Eigen::VectorXd>&>(
               &ampc::MPCBase::setStateWeights),
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

  base.def("setInputRateWeights", &ampc::MPCBase::setInputRateWeights,
           R"doc(
Set the weights of the input rate cost
sum_k (u_k - u_{k-1})^T R_rate (u_k - u_{k-1}) over the sampled inputs,
where u_{-1} is the previous input (see setPreviousInput()). Defaults to ones.

This method can only be called if `use_input_rate_cost` is enabled. Unlike `R`
(which weights the control points), the rate cost acts on the sampled inputs,
so it penalizes the same input change equally for every basis.

Args:
    R_rate_diag: Diagonal of the input rate weight matrix (size input_dim,
        non-negative).
           )doc",
           py::arg("R_rate_diag"));

  base.def("setReferenceState", &ampc::MPCBase::setReferenceState,
           R"doc(
Set reference state trajectory as a step command.

Args:
    x_step: Reference state vector to use for entire trajectory.

Note:
    The QP is updated in the next prepare() or solve(), which report update
    failures.
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

Note:
    The QP is updated in the next prepare() or solve(), which report update
    failures.
           )doc",
           py::arg("x_traj"));

  base.def("setReferenceInput", &ampc::MPCBase::setReferenceInput,
           R"doc(
Set reference input trajectory as a step command (reference for all control
points set to this value).

This method can only be called if `use_input_cost` is enabled.

Args:
    u_step: Reference input vector to use for entire trajectory.

Note:
    The QP is updated in the next prepare() or solve(), which report update
    failures.
           )doc",
           py::arg("u_step"));

  base.def("setReferenceInputControlPoints",
           &ampc::MPCBase::setReferenceInputControlPoints,
           R"doc(
Set reference control points that parameterize the reference input trajectory.

This method can only be called if `use_input_cost` is enabled.

Args:
    control_points (vector): Reference input control points as a stacked vector.
        Should have length of input_dim * num_control_points. If you have a
        matrix where each row is a control point, then you can pass in
        `control_points.ravel()` to convert it to a vector.

Note:
    The QP is updated in the next prepare() or solve(), which report update
    failures.
            )doc",
           py::arg("control_points"));

  base.def("setInputLimits", &ampc::MPCBase::setInputLimits,
           R"doc(
Set input saturation limits.

This function must be called prior to `initializeSolver()`, but can also be
called after if limits change between solves.

Args:
    u_min: Lower bound on input vector.
    u_max: Upper bound on input vector.

Note:
    The QP is updated in the next prepare() or solve(), which report update
    failures.
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

Note:
    The QP is updated in the next prepare() or solve(), which report update
    failures.
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
    control_point_slew: Slew-rate vector. All values must be positive.

Note:
    The QP is updated in the next prepare() or solve(), which report update
    failures.
           )doc",
           py::arg("control_point_slew"));

  base.def("setSlewRateInitial", &ampc::MPCBase::setSlewRateInitial,
           R"doc(
Set initial slew-rate constraint limits for control points. Initial input can
vary by up to this magnitude from the previous input. See `setPreviousInput()`.

This function can only be called if `slew_initial_input` is enabled. Must be
called prior to `initializeSolver()`, but can also be called after if limits
change between solves.

Args:
    u0_slew: Initial slew-rate vector. All values must be positive.

Note:
    The QP is updated in the next prepare() or solve(), which report update
    failures.
           )doc",
           py::arg("u0_slew"));

  base.def("setPreviousInput", &ampc::MPCBase::setPreviousInput,
           R"doc(
Set the previous input, which is used by the `slew_initial_input` constraint
and the `use_input_rate_cost` cost. Defaults to zeros, but should be set prior
to first solve if a different value is desired.

This value is automatically set to the next input after each solve that returns
`Success` or `SolvedInaccurate`, so you really only need to call this function
once before the first solve, or after a failed solve (which leaves it unchanged)
if you apply an input other than the stored one.

This function can only be called if `slew_initial_input` or
`use_input_rate_cost` is enabled.

Args:
    u_prev: Previous input vector.

Note:
    The QP is updated in the next prepare() or solve(), which report update
    failures.
           )doc",
           py::arg("u_prev"));

  base.def("getQPCostMatrix", &ampc::MPCBase::getQPCostMatrix,
           py::return_value_policy::copy,
           R"doc(
Get a copy of the QP cost matrix P (the Hessian of the QP objective), for
analysis and debugging.

Reflects the matrix used by the last solve() or prepare(): setters do not change
it, model or weight changes are applied by the next prepare() or solve(), and
SparseMPC holds unit placeholder weights between initializeSolver() and the
first prepare() or solve().

Returns:
    P (matrix): Copy of the full symmetric QP cost matrix.
           )doc");

  base.def("isSolverInitialized", &ampc::MPCBase::isSolverInitialized,
           "Whether initializeSolver() has succeeded.");

  // Configuration getters return copies so Python never holds a reference into
  // the MPC object.
  const auto copy{py::return_value_policy::copy};
  base.def("getParameterization", &ampc::MPCBase::getParameterization, copy,
           "Get a copy of the input trajectory parameterization.");
  base.def("getOptions", &ampc::MPCBase::getOptions, copy,
           "Get a copy of the options enabled at construction.");
  base.def("getStateWeights", &ampc::MPCBase::getStateWeights,
           "Get the diagonal of the state weight matrix Q (size state_dim).");
  base.def("getTerminalStateWeights", &ampc::MPCBase::getTerminalStateWeights,
           "Get the diagonal of the terminal state weight matrix Q_f (size "
           "state_dim).");
  base.def("getInputWeights", &ampc::MPCBase::getInputWeights,
           R"doc(
Get the diagonal of the input weight matrix R (size input_dim).

Raises:
    RuntimeError: If `use_input_cost` is not enabled.
           )doc");
  base.def("getInputRateWeights", &ampc::MPCBase::getInputRateWeights, copy,
           R"doc(
Get the diagonal of the input rate weight matrix R_rate (size input_dim).

Raises:
    RuntimeError: If `use_input_rate_cost` is not enabled.
           )doc");
  base.def("getInputMin", &ampc::MPCBase::getInputMin, copy,
           "Get the lower input limit (-inf until setInputLimits()).");
  base.def("getInputMax", &ampc::MPCBase::getInputMax, copy,
           "Get the upper input limit (+inf until setInputLimits()).");
  base.def("getStateMin", &ampc::MPCBase::getStateMin, copy,
           R"doc(
Get the lower state limit (-inf until setStateLimits()).

Raises:
    RuntimeError: If `saturate_states` is not enabled.
           )doc");
  base.def("getStateMax", &ampc::MPCBase::getStateMax, copy,
           R"doc(
Get the upper state limit (+inf until setStateLimits()).

Raises:
    RuntimeError: If `saturate_states` is not enabled.
           )doc");
  base.def("getSlewRate", &ampc::MPCBase::getSlewRate, copy,
           R"doc(
Get the control point slew rate (+inf until setSlewRate()).

Raises:
    RuntimeError: If `slew_control_points` is not enabled.
           )doc");
  base.def("getSlewRateInitial", &ampc::MPCBase::getSlewRateInitial, copy,
           R"doc(
Get the initial input slew rate (+inf until setSlewRateInitial()).

Raises:
    RuntimeError: If `slew_initial_input` is not enabled.
           )doc");
  base.def("getPreviousInput", &ampc::MPCBase::getPreviousInput, copy,
           R"doc(
Get the previous input used by the initial slew rate constraint and the input
rate cost. Updated automatically after each solve that returns `Success` or
`SolvedInaccurate`.

Raises:
    RuntimeError: If neither `slew_initial_input` nor `use_input_rate_cost` is
        enabled.
           )doc");
  base.def("getReferenceStateTrajectory",
           &ampc::MPCBase::getReferenceStateTrajectory, copy,
           "Get the reference state trajectory as stacked states (size "
           "state_dim * horizon_steps).");
  base.def("getReferenceInputControlPoints",
           &ampc::MPCBase::getReferenceInputControlPoints, copy,
           R"doc(
Get the reference input control points as a stacked vector (size
input_dim * num_control_points).

Raises:
    RuntimeError: If `use_input_cost` is not enabled.
           )doc");

  base.def("getSolveInfo", &ampc::MPCBase::getSolveInfo,
           R"doc(
Get solver diagnostics from the last solve() (status, iterations, objective,
and timing).

After a solve() that returned `UpdateFailed`, the solver fields still describe
the previous solve, while `mpc_solve_time` covers the failed call.
`prepare_time` covers the prepare() calls between the previous solve() and
the last one.

Returns:
    info (SolveInfo): Default values (status `NotInitialized`) before
        initializeSolver().
           )doc");

  base.def("resetWarmStart", &ampc::MPCBase::resetWarmStart,
           R"doc(
Make the next solve() start from zero instead of warm starting from the previous
solution (an OSQP cold start).

Useful after a failed solve, whose iterates may have diverged and would slow
down or prevent convergence of the next solve. Does not change the problem data,
the previous input used by `slew_initial_input`, or the results returned by the
solution getters. No-op before initializeSolver().
           )doc");

  base.def("isWithinSparsityPattern", &ampc::MPCBase::isWithinSparsityPattern,
           R"doc(
Debugging aid: check whether the current QP matrices fit the sparsity pattern
fixed by initializeSolver().

Model or weight updates that add nonzeros outside that pattern are silently
dropped by the solver, so the MPC solves a different problem than the one you
set. Call this after prepare() or solve() (setters do not update the QP) to
detect that. It scans every entry of the QP matrices, so avoid calling it in
time-critical loops.

Returns:
    within_pattern (bool): True if the QP matrices fit the pattern, or if the
        solver is not initialized yet. False otherwise.
           )doc");

  base.def_property_readonly("state_dim", &ampc::MPCBase::getStateDim);
  base.def_property_readonly("input_dim", &ampc::MPCBase::getInputDim);
  base.def_property_readonly("horizon_steps", &ampc::MPCBase::getHorizonSteps);
  base.def_property_readonly("num_control_points",
                             &ampc::MPCBase::getNumControlPoints);

  // MPCBase can't work in Python anyways because derived classes resize Eigen
  // variables base.def("_qpUpdateX0", static_cast<void (MPCBase::*)(const
  // Eigen::Ref<const Eigen::VectorXd>&)>(&PyMPCBase::qpUpdateX0),
  //   "PRIVATE - do not call manually! Defines how to convert MPC problem to a
  //   QP problem", py::arg("x0"));

  base.def("__str__", [](const ampc::MPCBase& self) {
    std::ostringstream oss;
    const bool capitalize_bools{true};
    ampc::print(oss, self, capitalize_bools);
    return oss.str();
  });

  base.def("__repr__", [](const ampc::MPCBase& self) {
    std::ostringstream oss;
    const bool capitalize_bools{true};
    ampc::printInline(oss, self, capitalize_bools);
    return oss.str();
  });
}

} // namespace affine_mpc_py
