#include "affine_mpc_py_module.hpp"

#include <pybind11/eigen.h>
#include <pybind11/native_enum.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <sstream>
#include <stdexcept>
#include <string>

#include "affine_mpc/mpc.hpp"
#include "affine_mpc/options.hpp"
#include "affine_mpc/parameterization.hpp"

namespace affine_mpc_py {
namespace ampc = affine_mpc;
namespace py = pybind11;

// The C++ solution getters do not check initialization (hot path), but reading
// the solution before initializeSolver() dereferences a null pointer, so guard
// them here to raise a Python exception instead of crashing the interpreter.
static void requireSolverInitialized(const ampc::MPC& self,
                                     const char* func_name)
{
  if (!self.isSolverInitialized())
    throw std::logic_error(std::string{"[MPC::"} + func_name
                           + "] Solver must be initialized first.");
}

void moduleAddMPC(py::module& m)
{
  py::native_enum<ampc::Formulation>(m, "Formulation", "enum.Enum",
                                     R"doc(
How the QP represents the predicted states.

Both formulations solve the same MPC problem and give the same solution (up to
the solver tolerance); they differ in the size and sparsity of the QP, and so
in solve time.
)doc")
      .value("Condensed", ampc::Formulation::Condensed,
             "The states are eliminated with the model, so only the control "
             "points are QP variables: a small, dense QP. Preferred for "
             "shorter horizons and lower-dimensional problems, and the "
             "default.")
      .value("Sparse", ampc::Formulation::Sparse,
             "The predicted states are QP variables too, and the model is a "
             "set of equality constraints: a larger, sparse QP. Preferred for "
             "longer horizons or larger state dimensions.")
      .finalize();

  py::class_<ampc::MPC> base(m, "MPC", R"doc(
Model predictive control of an affine model, solved as a QP with OSQP.

Optimizes the control points of an input trajectory parameterization
(B-splines, move-blocking, or a custom basis) subject to the costs and
constraints enabled in Options. The formulation (condensed or sparse) chooses
how the QP represents the predicted states; it is fixed at construction and
does not change the interface.

Setters only validate and store their values; all QP work happens in solve(),
or earlier in the optional prepare().
)doc");
  base.def(py::init<const int, const int, const ampc::Parameterization&,
                    const ampc::Options&, ampc::Formulation>(),
           R"doc(
Construct an MPC with an input parameterization, options, and formulation.

Args:
    state_dim: State vector dimension.
    input_dim: Input vector dimension.
    param: Input trajectory parameterization.
    opts: Optional MPC configuration features to enable.
    formulation: How the QP represents the predicted states (keyword only).

Raises:
    ValueError: If a dimension is not positive or the options are invalid for
        the dimensions or the parameterization.
           )doc",
           py::arg("state_dim"), py::arg("input_dim"), py::arg("param"),
           py::arg("opts") = ampc::Options{}, py::kw_only(),
           py::arg("formulation") = ampc::Formulation::Condensed);
  base.def(py::init<const int, const int, const int, const ampc::Options&,
                    ampc::Formulation>(),
           R"doc(
Construct an MPC that optimizes every input of the horizon (no
parameterization: move-blocking with one control point per step).

Args:
    state_dim: State vector dimension.
    input_dim: Input vector dimension.
    horizon_steps: Number of discrete time steps in the horizon.
    opts: Optional MPC configuration features to enable.
    formulation: How the QP represents the predicted states (keyword only).
           )doc",
           py::arg("state_dim"), py::arg("input_dim"), py::arg("horizon_steps"),
           py::arg("opts") = ampc::Options{}, py::kw_only(),
           py::arg("formulation") = ampc::Formulation::Condensed);

  base.def("initializeSolver",
           static_cast<bool (ampc::MPC::*)(const OSQPSettings&,
                                           ampc::ModelSparsity)>(
               &ampc::MPC::initializeSolver),
           R"doc(
Initialize OSQP solver after configuring MPC setup. Calling this method after
a successful initialization will simply return True without doing anything.

Prior to calling this function, you must have set the model and input limits.
You must also provide parameters for all enabled options before calling this
function. For example, if state saturation is enabled then `setStateLimits()`
must be called beforehand.

Initialization fixes the solver's sparsity pattern; later updates only change
the values of the entries it tracks. `model_sparsity` chooses the tracked
entries of Ad and Bd: with the default ModelSparsity.InitialNonzeros, the
entries nonzero in the model set at this call, so an entry that may become
nonzero later must be nonzero now (set a representative model before this call
and the actual model after it), and every later model must be zero outside them
(a nonzero outside them is silently dropped; see isWithinSparsityPattern()).
With ModelSparsity.Full, every entry, which costs more per solve when the model
has structural zeros. The condensed formulation supports only
ModelSparsity.InitialNonzeros for now.

Args:
    solver_settings: OSQP solver settings. Defaults to recommended settings.
    model_sparsity: Which entries of Ad and Bd the solver tracks.

Returns:
    success: True if initialization succeeds, false otherwise.

Raises:
    RuntimeError: If a setter the solver or the options require has not been
        called.
    ValueError: For ModelSparsity.Full with the condensed formulation.
           )doc",
           py::arg("solver_settings") =
               ampc::OSQPSolver::getRecommendedSettings(),
           py::arg("model_sparsity") = ampc::ModelSparsity::InitialNonzeros);

  base.def("solve", &ampc::MPC::solve,
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

  base.def("prepare", &ampc::MPC::prepare,
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
      [](ampc::MPC& self, Eigen::Ref<Eigen::VectorXd> u0) {
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
      [](ampc::MPC& self) {
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
      [](ampc::MPC& self, Eigen::Ref<Eigen::VectorXd> control_points) {
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
      [](ampc::MPC& self) {
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
      [](ampc::MPC& self, Eigen::Ref<Eigen::VectorXd> u_traj) {
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
      [](ampc::MPC& self) {
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
      [](ampc::MPC& self, Eigen::Ref<Eigen::VectorXd> x_traj) {
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
      [](ampc::MPC& self) {
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
      [](ampc::MPC& self, const Eigen::Ref<const Eigen::VectorXd>& x,
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
      [](ampc::MPC& self, const Eigen::Ref<const Eigen::VectorXd>& x,
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

  base.def("setModelDiscrete", &ampc::MPC::setModelDiscrete,
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
           &ampc::MPC::setModelContinuous2Discrete,
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

  base.def("setModelOffsetDiscrete", &ampc::MPC::setModelOffsetDiscrete,
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
           &ampc::MPC::setModelOffsetContinuous2Discrete,
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
               &ampc::MPC::setWeights),
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
               &ampc::MPC::setWeights),
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
               &ampc::MPC::setStateWeights),
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
               &ampc::MPC::setStateWeights),
           R"doc(
Set only the state and terminal state weights for the cost function.

Args:
    Q_diag: Vector for diagonal of state weight matrix (non-negative).
    Qf_diag: Vector for diagonal of terminal state weight matrix (non-negative).
      )doc",
           py::arg("Q_diag"), py::arg("Qf_diag"));

  base.def("setInputWeights", &ampc::MPC::setInputWeights,
           R"doc(
Set only the input weights for the cost function.

This method can only be called if `use_input_cost` is enabled.

Args:
    R_diag: Vector for diagonal of input weight matrix (non-negative).
           )doc",
           py::arg("R_diag"));

  base.def("setInputRateWeights", &ampc::MPC::setInputRateWeights,
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

  base.def("setReferenceState", &ampc::MPC::setReferenceState,
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
           &ampc::MPC::setReferenceStateTrajectory,
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

  base.def("setReferenceInput", &ampc::MPC::setReferenceInput,
           R"doc(
Set reference input trajectory as a step command (reference for all control
points set to this value).

This method can only be called if `use_input_cost` is enabled, and not with a
custom basis (Parameterization.fromBasis()), whose coefficients do not
represent a constant input; use setReferenceInputControlPoints() there.

Args:
    u_step: Reference input vector to use for entire trajectory.

Note:
    The QP is updated in the next prepare() or solve(), which report update
    failures.
           )doc",
           py::arg("u_step"));

  base.def("setReferenceInputControlPoints",
           &ampc::MPC::setReferenceInputControlPoints,
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

  base.def("setInputLimits", &ampc::MPC::setInputLimits,
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
  base.def("setStateLimits", &ampc::MPC::setStateLimits,
           R"doc(
Set state saturation limits.

This function can only be called if `saturate_states` is enabled. Must be
called prior to `initializeSolver()`, but can also be called after if limits
change between solves. The limits have one entry per bounded state
(Options.bounded_states, all states by default); pass `-inf` or `inf` for a side
without a limit.

Args:
    x_min: Lower limits of the bounded states.
    x_max: Upper limits of the bounded states.

Raises:
    ValueError: If a size is not the number of bounded states or x_min > x_max.

Note:
    The QP is updated in the next prepare() or solve(), which report update
    failures.
           )doc",
           py::arg("x_min"), py::arg("x_max"));

  base.def("setStateBoundPenalties", &ampc::MPC::setStateBoundPenalties,
           R"doc(
Set the penalties of the soft state limits: the cost gains
l1^T s + s^T diag(l2) s, where s >= 0 holds one slack per bounded state (shared
over the horizon) that may widen both of its limits.

This function can only be called if `soft_state_bounds` is enabled, and must be
called prior to `initializeSolver()`. The L1 penalty is exact: when it exceeds
the limit's Lagrange multiplier, the limits are only violated when they can not
be met. The L2 penalty alone always allows a small violation; with the L1
penalty it does not change the solution while the limits can be met. OSQP
converges much faster with an L2 penalty on the scale of the state weights or
larger (with a tiny one, soft limits can take 10 to 30 times the iterations of
hard limits), and slower with a very large L1 penalty. There is no default: a
large enough L1 penalty depends on the problem.

Args:
    l1: Linear penalties, one per bounded state (non-negative).
    l2: Quadratic penalties, one per bounded state (non-negative).

Raises:
    RuntimeError: If `soft_state_bounds` is not enabled.
    ValueError: If a size is not the number of bounded states or a penalty is
        negative.
           )doc",
           py::arg("l1"), py::arg("l2"));
  base.def("setGeneralConstraints", &ampc::MPC::setGeneralConstraints,
           R"doc(
Set the general linear constraints h_min <= Hx x_k + Hu u_k <= h_max, applied at
every step k = 0, ..., T-1.

This function can only be called if `num_general_constraints` (p) is positive.
Must be called prior to `initializeSolver()`, but can also be called after if
the constraints change between solves. x_0 is the measured state passed to
solve(), so the k = 0 rows constrain u_0 given x_0; a row without an input part
(a zero row of Hu) only checks x_0 at k = 0, and the problem is infeasible if x_0
violates it. The predicted states x_1, ..., x_{T-1} are paired with the inputs
u_1, ..., u_{T-1}; constrain the final state x_T with setTerminalConstraints().
Pass `-inf` or `inf` for a side without a bound.

Changing Hx or Hu rebuilds QP matrices and makes OSQP refactor (like a model
change), while changing only the bounds updates QP vectors: Hx and Hu count as
changed only when a value differs from the stored one. With the condensed
formulation, as with the model, an entry of Hx or Hu that may become nonzero
later must be nonzero when initializeSolver() is called, since that fixes the
solver's sparsity pattern; the sparse formulation keeps every entry of Hx and Hu
in its pattern.

Args:
    Hx: State matrix (p x state_dim).
    Hu: Input matrix (p x input_dim).
    h_min: Lower bounds (size p).
    h_max: Upper bounds (size p).

Raises:
    RuntimeError: If `num_general_constraints` is zero.
    ValueError: If a shape is wrong or h_min > h_max.

Note:
    The QP is updated in the next prepare() or solve(), which report update
    failures.
           )doc",
           py::arg("Hx"), py::arg("Hu"), py::arg("h_min"), py::arg("h_max"));
  base.def("setTerminalConstraints", &ampc::MPC::setTerminalConstraints,
           R"doc(
Set the terminal constraints hf_min <= Hf x_T <= hf_max on the final predicted
state.

This function can only be called if `num_terminal_constraints` (p_T) is
positive. Must be called prior to `initializeSolver()`, but can also be called
after if the constraints change between solves. Pass `-inf` or `inf` for a side
without a bound. As with setGeneralConstraints(), changing Hf rebuilds QP
matrices and makes OSQP refactor, changing only the bounds updates QP vectors,
and with the condensed formulation an entry of Hf that may become nonzero later
must be nonzero when initializeSolver() is called (the sparse formulation keeps
every entry of Hf in its pattern).

Args:
    Hf: State matrix (p_T x state_dim).
    hf_min: Lower bounds (size p_T).
    hf_max: Upper bounds (size p_T).

Raises:
    RuntimeError: If `num_terminal_constraints` is zero.
    ValueError: If a shape is wrong or hf_min > hf_max.

Note:
    The QP is updated in the next prepare() or solve(), which report update
    failures.
           )doc",
           py::arg("Hf"), py::arg("hf_min"), py::arg("hf_max"));
  base.def("setSlewRate", &ampc::MPC::setSlewRate,
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

  base.def("setSlewRateInitial", &ampc::MPC::setSlewRateInitial,
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

  base.def("setPreviousInput", &ampc::MPC::setPreviousInput,
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

  base.def("getQPCostMatrix", &ampc::MPC::getQPCostMatrix,
           py::return_value_policy::copy,
           R"doc(
Get a copy of the QP cost matrix P (the Hessian of the QP objective), for
analysis and debugging.

Reflects the matrix used by the last solve() or prepare(): setters do not change
it, and model or weight changes are applied by the next prepare() or solve().

The sparse formulation stores P in sparse form, so each call also builds the
dense matrix; not intended for the control loop.

The solver reads only the upper triangle. For the condensed formulation this is
the stored P, built by matrix products and so symmetric only up to round-off;
for the sparse formulation the lower triangle is mirrored from the upper one,
so it is exactly symmetric.
getQP() returns an exactly symmetric P for both.

Returns:
    P (matrix): Copy of the full QP cost matrix.
           )doc");

  base.def("getQP", &ampc::MPC::getQP,
           R"doc(
Analysis aid: get the QP as last written by solve(), prepare(), or assembleQP(),
for analysis, debugging, or handing to another solver.

Setters do not change it; to get the QP for the current settings and a given
initial state without solving, use assembleQP(). Copies the QP, so not intended
for the control loop.

Returns:
    qp (QuadraticProgram): The QP; reading its P or A needs SciPy.
           )doc");

  base.def("assembleQP", &ampc::MPC::assembleQP,
           R"doc(
Analysis aid: assemble the QP for the current settings and the initial state x0
without solving it, for analysis, debugging, or handing to another solver.

Works before or after initializeSolver() and needs the same setup (model, input
limits, and the setters the options require). It does not send anything to the
solver: the parts it writes are sent by the next prepare() or solve(), which
therefore repeats the matrix work once. Not intended for the control loop.

Args:
    x0 (vector): Initial state (state_dim).
    qp (QuadraticProgram): Output QP, e.g. QuadraticProgram(); filled in place.

Raises:
    RuntimeError: If the setup required by initializeSolver() is missing.
           )doc",
           py::arg("x0"), py::arg("qp"));

  base.def("getQPVariableBlocks", &ampc::MPC::getQPVariableBlocks,
           R"doc(
Get the blocks of QP variables z, in order: "control_points" (input
coefficients), then for the sparse formulation "states" (x_1, ..., x_T,
state_dim each), then with soft state bounds "state_slacks" (one per bounded
state). Together they cover every variable. For analysis; allocates the list,
so not intended for the control loop.

Returns:
    blocks (list[QPBlock]): The variable blocks in order.
           )doc");

  base.def("getQPConstraintBlocks", &ampc::MPC::getQPConstraintBlocks,
           R"doc(
Get the blocks of constraint rows of A, in order, listing only the blocks the
options enable: "dynamics" (the sparse formulation's model rows),
"input_bounds", "initial_slew", "control_point_slew", "state_bounds" (soft: the
upper rows, the lower rows, then the slack rows s >= 0), "general_constraints"
(rows per step, steps in order), and "terminal_constraints". Together they
cover every row. For analysis; allocates the list, so not intended for the
control loop.

Returns:
    blocks (list[QPBlock]): The constraint row blocks in order.
           )doc");

  base.def("getNumQPVariables", &ampc::MPC::getNumQPVariables,
           "Get the number of QP variables (the size of z).");
  base.def("getNumQPConstraints", &ampc::MPC::getNumQPConstraints,
           "Get the number of QP constraint rows (the rows of A).");

  base.def("getModelSparsity", &ampc::MPC::getModelSparsity,
           "The model sparsity passed to initializeSolver() (the default, "
           "ModelSparsity.InitialNonzeros, before it is called).");
  base.def("isSolverInitialized", &ampc::MPC::isSolverInitialized,
           "Whether initializeSolver() has succeeded.");

  // Configuration getters return copies so Python never holds a reference into
  // the MPC object.
  const auto copy{py::return_value_policy::copy};
  base.def("getParameterization", &ampc::MPC::getParameterization, copy,
           "Get a copy of the input trajectory parameterization.");
  base.def("getOptions", &ampc::MPC::getOptions, copy,
           "Get a copy of the options enabled at construction.");
  base.def("getStateWeights", &ampc::MPC::getStateWeights,
           "Get the diagonal of the state weight matrix Q (size state_dim).");
  base.def("getTerminalStateWeights", &ampc::MPC::getTerminalStateWeights,
           "Get the diagonal of the terminal state weight matrix Q_f (size "
           "state_dim).");
  base.def("getInputWeights", &ampc::MPC::getInputWeights,
           R"doc(
Get the diagonal of the input weight matrix R (size input_dim).

Raises:
    RuntimeError: If `use_input_cost` is not enabled.
           )doc");
  base.def("getInputRateWeights", &ampc::MPC::getInputRateWeights, copy,
           R"doc(
Get the diagonal of the input rate weight matrix R_rate (size input_dim).

Raises:
    RuntimeError: If `use_input_rate_cost` is not enabled.
           )doc");
  base.def("getInputMin", &ampc::MPC::getInputMin, copy,
           "Get the lower input limit (-inf until setInputLimits()).");
  base.def("getInputMax", &ampc::MPC::getInputMax, copy,
           "Get the upper input limit (+inf until setInputLimits()).");
  base.def("getStateMin", &ampc::MPC::getStateMin, copy,
           R"doc(
Get the lower limits of the bounded states (-inf until setStateLimits()).

Raises:
    RuntimeError: If `saturate_states` is not enabled.
           )doc");
  base.def("getStateMax", &ampc::MPC::getStateMax, copy,
           R"doc(
Get the upper limits of the bounded states (+inf until setStateLimits()).

Raises:
    RuntimeError: If `saturate_states` is not enabled.
           )doc");
  base.def("getBoundedStates", &ampc::MPC::getBoundedStates,
           R"doc(
Get the indices of the bounded states (Options.bounded_states, or every state
when it is empty; none without `saturate_states`).
           )doc");
  base.def("getStateBoundL1Penalties", &ampc::MPC::getStateBoundL1Penalties,
           copy,
           R"doc(
Get the L1 penalties of the soft state limits (zero until
setStateBoundPenalties()).

Raises:
    RuntimeError: If `soft_state_bounds` is not enabled.
           )doc");
  base.def("getStateBoundL2Penalties", &ampc::MPC::getStateBoundL2Penalties,
           copy,
           R"doc(
Get the L2 penalties of the soft state limits (zero until
setStateBoundPenalties()).

Raises:
    RuntimeError: If `soft_state_bounds` is not enabled.
           )doc");
  base.def("getGeneralConstraintStateMatrix",
           &ampc::MPC::getGeneralConstraintStateMatrix, copy,
           R"doc(
Get the state matrix Hx of the general constraints (zero until
setGeneralConstraints()).

Raises:
    RuntimeError: If `num_general_constraints` is zero.
           )doc");
  base.def("getGeneralConstraintInputMatrix",
           &ampc::MPC::getGeneralConstraintInputMatrix, copy,
           R"doc(
Get the input matrix Hu of the general constraints (zero until
setGeneralConstraints()).

Raises:
    RuntimeError: If `num_general_constraints` is zero.
           )doc");
  base.def("getGeneralConstraintMin", &ampc::MPC::getGeneralConstraintMin, copy,
           R"doc(
Get the lower bounds of the general constraints (-inf until
setGeneralConstraints()).

Raises:
    RuntimeError: If `num_general_constraints` is zero.
           )doc");
  base.def("getGeneralConstraintMax", &ampc::MPC::getGeneralConstraintMax, copy,
           R"doc(
Get the upper bounds of the general constraints (+inf until
setGeneralConstraints()).

Raises:
    RuntimeError: If `num_general_constraints` is zero.
           )doc");
  base.def("getTerminalConstraintMatrix",
           &ampc::MPC::getTerminalConstraintMatrix, copy,
           R"doc(
Get the state matrix Hf of the terminal constraints (zero until
setTerminalConstraints()).

Raises:
    RuntimeError: If `num_terminal_constraints` is zero.
           )doc");
  base.def("getTerminalConstraintMin", &ampc::MPC::getTerminalConstraintMin,
           copy,
           R"doc(
Get the lower bounds of the terminal constraints (-inf until
setTerminalConstraints()).

Raises:
    RuntimeError: If `num_terminal_constraints` is zero.
           )doc");
  base.def("getTerminalConstraintMax", &ampc::MPC::getTerminalConstraintMax,
           copy,
           R"doc(
Get the upper bounds of the terminal constraints (+inf until
setTerminalConstraints()).

Raises:
    RuntimeError: If `num_terminal_constraints` is zero.
           )doc");
  base.def("getSlewRate", &ampc::MPC::getSlewRate, copy,
           R"doc(
Get the control point slew rate (+inf until setSlewRate()).

Raises:
    RuntimeError: If `slew_control_points` is not enabled.
           )doc");
  base.def("getSlewRateInitial", &ampc::MPC::getSlewRateInitial, copy,
           R"doc(
Get the initial input slew rate (+inf until setSlewRateInitial()).

Raises:
    RuntimeError: If `slew_initial_input` is not enabled.
           )doc");
  base.def("getPreviousInput", &ampc::MPC::getPreviousInput, copy,
           R"doc(
Get the previous input used by the initial slew rate constraint and the input
rate cost. Updated automatically after each solve that returns `Success` or
`SolvedInaccurate`.

Raises:
    RuntimeError: If neither `slew_initial_input` nor `use_input_rate_cost` is
        enabled.
           )doc");
  base.def("getReferenceStateTrajectory",
           &ampc::MPC::getReferenceStateTrajectory, copy,
           "Get the reference state trajectory as stacked states (size "
           "state_dim * horizon_steps).");
  base.def("getReferenceInputControlPoints",
           &ampc::MPC::getReferenceInputControlPoints, copy,
           R"doc(
Get the reference input control points as a stacked vector (size
input_dim * num_control_points).

Raises:
    RuntimeError: If `use_input_cost` is not enabled.
           )doc");

  base.def("getSolveInfo", &ampc::MPC::getSolveInfo,
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

  base.def("resetWarmStart", &ampc::MPC::resetWarmStart,
           R"doc(
Make the next solve() start from zero instead of warm starting from the previous
solution (an OSQP cold start).

Useful after a failed solve, whose iterates may have diverged and would slow
down or prevent convergence of the next solve. Does not change the problem data,
the previous input used by `slew_initial_input`, or the results returned by the
solution getters. No-op before initializeSolver().
           )doc");

  base.def("isWithinSparsityPattern", &ampc::MPC::isWithinSparsityPattern,
           R"doc(
Debugging aid: check whether the current QP matrices fit the sparsity pattern
fixed by initializeSolver().

Model, weight, or constraint matrix updates that add nonzeros outside that
pattern are silently dropped by the solver, so the MPC solves a different
problem than the one you set. Call this after prepare() or solve() (setters do
not update the QP) to detect that. The sparse formulation's QP holds only the
entries it tracks, so it always fits OSQP's pattern; instead, the model must be
zero outside the tracked entries of Ad and Bd (see ModelSparsity), which this
also checks. It scans the QP matrices, so avoid calling it in time-critical loops.

Returns:
    within_pattern (bool): True if the model and the QP matrices fit the
        pattern, or if the solver is not initialized yet. False otherwise.
           )doc");

  base.def("getFormulation", &ampc::MPC::getFormulation,
           "Get the formulation chosen at construction.");

  base.def_property_readonly("state_dim", &ampc::MPC::getStateDim);
  base.def_property_readonly("input_dim", &ampc::MPC::getInputDim);
  base.def_property_readonly("horizon_steps", &ampc::MPC::getHorizonSteps);
  base.def_property_readonly("num_control_points",
                             &ampc::MPC::getNumControlPoints);

  base.def("__str__", [](const ampc::MPC& self) {
    std::ostringstream oss;
    const bool capitalize_bools{true};
    ampc::print(oss, self, capitalize_bools);
    return oss.str();
  });

  base.def("__repr__", [](const ampc::MPC& self) {
    std::ostringstream oss;
    const bool capitalize_bools{true};
    ampc::printInline(oss, self, capitalize_bools);
    return oss.str();
  });
}

} // namespace affine_mpc_py
