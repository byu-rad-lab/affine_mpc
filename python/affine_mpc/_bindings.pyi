"""
Affine MPC module
"""

import numpy
import numpy.typing
import typing
import os

__all__: list[str] = [
    "CondensedMPC",
    "MPCBase",
    "MPCLogger",
    "OSQPSettings",
    "Options",
    "Parameterization",
    "SolveStatus",
    "SparseMPC",
]

class CondensedMPC(MPCBase):
    """

    MPC formulation where only parameterization control points are optimization
    design variables (condensed QP).

    Eliminates state variables analytically by implicitly wrapping the model into
    the const function rather than as a constraint , resulting in a smaller dense
    QP. Preferred for shorter horizons and lower-dimensional problems.
    Converts the MPC problem to QP form for OSQP, using input parameterization.

    """

    @typing.overload
    def __init__(
        self,
        state_dim: int,
        input_dim: int,
        param: Parameterization,
        opts: Options = Options(),
    ) -> None:
        """
        Construct CondensedMPC with specified input parameterization and MPC
        configuration options.

        Args:
           state_dim: State vector dimension.
           input_dim: Input vector dimension.
           param: Input trajectory parameterization.
           opts: Optional MPC configuration features to enable.
        """

    @typing.overload
    def __init__(
        self,
        state_dim: int,
        input_dim: int,
        horizon_steps: int,
        opts: Options = Options(),
    ) -> None:
        """
        Construct CondensedMPC with no parameterization (full input trajectory will be
        optimized) and MPC configuration options.

        Args:
           state_dim: State vector dimension.
           input_dim: Input vector dimension.
           horizon_steps: Number of discrete steps in the MPC horizon.
           opts: Optional MPC configuration features to enable.
        """

class MPCBase:
    """
    Abstract class. Not usable on its own.
    """

    def __init__(
        self,
        state_dim: int,
        input_dim: int,
        param: Parameterization,
        opts: Options,
        num_design_vars: int,
        num_custom_constraints: int,
    ) -> None:
        """
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
        """

    @typing.overload
    def getInputControlPoints(
        self,
        control_points: typing.Annotated[
            numpy.typing.NDArray[numpy.float64], "[m, 1]", "flags.writeable"
        ],
    ) -> typing.Annotated[
        numpy.typing.NDArray[numpy.float64], "[m, 1]", "flags.writeable"
    ]:
        """
        Get the control points that parameterize the input trajectory from the previous
        solve.

        Args:
            control_points (vector): Result will be stored here (equivalent to return
                value).

        Returns:
            control_points (vector): The stacked control points.
        """

    @typing.overload
    def getInputControlPoints(
        self,
    ) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"]:
        """
        Get the control points that parameterize the input trajectory from the previous
        solve.

        Returns:
            control_points (vector): The stacked control points.
        """

    @typing.overload
    def getInputTrajectory(
        self,
        u_traj: typing.Annotated[
            numpy.typing.NDArray[numpy.float64], "[m, 1]", "flags.writeable"
        ],
    ) -> typing.Annotated[
        numpy.typing.NDArray[numpy.float64], "[m, 1]", "flags.writeable"
    ]:
        """
        Get the full input trajectory from the previous solve.

        Args:
            u_traj (vector): Result will be stored here (equivalent to return value).

        returns:
            u_traj (vector): The input trajectory.
        """

    @typing.overload
    def getInputTrajectory(
        self,
    ) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"]:
        """
        Get the full input trajectory from the previous solve.

        returns:
            u_traj (vector): The input trajectory.
        """

    @typing.overload
    def getNextInput(
        self,
        u0: typing.Annotated[
            numpy.typing.NDArray[numpy.float64], "[m, 1]", "flags.writeable"
        ],
    ) -> typing.Annotated[
        numpy.typing.NDArray[numpy.float64], "[m, 1]", "flags.writeable"
    ]:
        """
        Get the next input to apply (initial input from optimized trajectory) from the
        previous solve.

        Args:
            u0: Result will be stored here (equivalent to return value).

        Returns:
            u0: Initial input from optimized trajectory (next to apply).
        """

    @typing.overload
    def getNextInput(
        self,
    ) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"]:
        """
        Get the next input to apply (initial input from optimized trajectory) from the
        previous solve.

        returns:
            u0: Initial input from optimized trajectory (next to apply).
        """

    @typing.overload
    def getPredictedStateTrajectory(
        self,
        x_traj: typing.Annotated[
            numpy.typing.NDArray[numpy.float64], "[m, 1]", "flags.writeable"
        ],
    ) -> typing.Annotated[
        numpy.typing.NDArray[numpy.float64], "[m, 1]", "flags.writeable"
    ]:
        """
        Get the predicted state trajectory from the previous solve.

        Args:
            x_traj (vector): Result will be stored here (equivalent to return value).

        returns:
            x_traj (vector): The predicted state trajectory.
        """

    @typing.overload
    def getPredictedStateTrajectory(
        self,
    ) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"]:
        """
        Get the predicted state trajectory from the previous solve.

        returns:
            x_traj (vector): The predicted state trajectory.
        """

    def initializeSolver(self, solver_settings: OSQPSettings = OSQPSettings()) -> bool:
        """
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
        """

    @typing.overload
    def propagateModel(
        self,
        x: typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"],
        u: typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"],
        x_next: typing.Annotated[
            numpy.typing.NDArray[numpy.float64], "[m, 1]", "flags.writeable"
        ],
    ) -> typing.Annotated[
        numpy.typing.NDArray[numpy.float64], "[m, 1]", "flags.writeable"
    ]:
        """
        Propagate the internal discrete-time model for one step.

        Model must be set prior to calling this method.

        Args:
            x: Current state vector.
            u: Current input vector.
            x_next (out): Resulting next state vector (equivalent to return value).

        Returns:
            x_next : Resulting next state vector.
        """

    @typing.overload
    def propagateModel(
        self,
        x: typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"],
        u: typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"],
    ) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"]:
        """
        Propagate the internal discrete-time model for one step.

        Model must be set prior to calling this method.

        Args:
            x: Current state vector.
            u: Current input vector.

        Returns:
            x_next : Resulting next state vector.
        """

    def setInputLimits(
        self,
        u_min: typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"],
        u_max: typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"],
    ) -> bool:
        """
        Set input saturation limits.

        This function must be called prior to `initializeSolver()`, but can also be
        called after if limits change between solves.

        Args:
            u_min: Lower bound on input vector.
            u_max: Upper bound on input vector.

        Returns:
            success: True if the internal QP was updated properly. Unlikely to be False.
        """

    def setInputWeights(
        self, R_diag: typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"]
    ) -> None:
        """
        Set only the input weights for the cost function.

        This method can only be called if `use_input_cost` is enabled.

        Args:
            R_diag: Vector for diagonal of input weight matrix (non-negative).
        """

    def setModelContinuous2Discrete(
        self,
        Ac: typing.Annotated[
            numpy.typing.NDArray[numpy.float64], "[m, n]", "flags.f_contiguous"
        ],
        Bc: typing.Annotated[
            numpy.typing.NDArray[numpy.float64], "[m, n]", "flags.f_contiguous"
        ],
        wc: typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"],
        dt: float,
        tol: float = 1e-06,
    ) -> bool:
        """
        Set the internal discrete-time model from a continuous-time model, which will be
        discretized. The model being `x_next = Ax + Bu + w`.

        The discretization assumes the input u is constant over the time step (true for
        discrete controllers like MPC) and uses a matrix exponential, which is an exact
        discretization (theoretically). The matrix exponential involves a Taylor series
        expansion $\\sum_{i=0}^\\inf (A*dt)^i / i!$. Thus the scalar term is `dt^i / i!`.
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
        """

    def setModelDiscrete(
        self,
        Ad: typing.Annotated[
            numpy.typing.NDArray[numpy.float64], "[m, n]", "flags.f_contiguous"
        ],
        Bd: typing.Annotated[
            numpy.typing.NDArray[numpy.float64], "[m, n]", "flags.f_contiguous"
        ],
        wd: typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"],
    ) -> bool:
        """
        Set the internal discrete-time model directly from a discrete model. The model
        being `x_next = Ax + Bu + w`.

        Args:
            Ad: Discrete-time state matrix A.
            Bd: Discrete-time input matrix B.
            wd: Discrete-time affine/bias vector w.

        Returns:
            success: True if the internal QP was updated properly. Unlikely to be False.
        """

    def setPreviousInput(
        self, u_prev: typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"]
    ) -> bool:
        """
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
        """

    def setReferenceInput(
        self, u_step: typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"]
    ) -> bool:
        """
        Set reference input trajectory as a step command (reference for all control
        points set to this value).

        This method can only be called if `use_input_cost` is enabled.

        Args:
            u_step: Reference input vector to use for entire trajectory.

        Returns:
            success: True if the internal QP was updated properly. Unlikely to be False.
        """

    def setReferenceInputControlPoints(
        self,
        control_points: typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"],
    ) -> bool:
        """
        Set reference control points that parameterize the reference input trajectory.

        This method can only be called if `use_input_cost` is enabled.

        Args:
            control_points (vector): Reference input control points as a stacked vector.
                Should have length of input_dim * num_control_points. If you have a
                matrix where each row is a control point, then you can pass in
                `control_points.ravel()` to convert it to a vector.

        Returns:
            success: True if the internal QP was updated properly. Unlikely to be False.
        """

    def setReferenceState(
        self, x_step: typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"]
    ) -> bool:
        """
        Set reference state trajectory as a step command.

        Args:
            x_step: Reference state vector to use for entire trajectory.

        Returns:
            success: True if the internal QP was updated properly. Unlikely to be False.
        """

    def setReferenceStateTrajectory(
        self, x_traj: typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"]
    ) -> bool:
        """
        Set reference state trajectory.

        Args:
            x_traj (vector): Reference state trajectory as a vector of stacked states.
                Should have length of state_dim * horizon_steps. If you have a matrix
                where each row is the state at index k in the horizon, then you can pass
                in `x_traj.ravel()` to convert it to a vector.

        Returns:
            success: True if the internal QP was updated properly. Unlikely to be False.
        """

    def setSlewRate(
        self,
        control_point_slew: typing.Annotated[
            numpy.typing.NDArray[numpy.float64], "[m, 1]"
        ],
    ) -> bool:
        """
        Set slew-rate constraint limits for control points. Control points can vary by
        up to this magnitude from one index to the next.

        This function can only be called if `slew_control_points` is enabled. Must be
        called prior to `initializeSolver()`, but can also be called after if limits
        change between solves.

        Args:
            control_point_slew: Slew-rate vector. All values must be positive.

        Returns:
            success: True if the internal QP was updated properly. Unlikely to be False.
        """

    def setSlewRateInitial(
        self, u0_slew: typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"]
    ) -> bool:
        """
        Set initial slew-rate constraint limits for control points. Initial input can
        vary by up to this magnitude from the previous input. See `setPreviousInput()`.

        This function can only be called if `slew_initial_input` is enabled. Must be
        called prior to `initializeSolver()`, but can also be called after if limits
        change between solves.

        Args:
            u0_slew: Initial slew-rate vector. All values must be positive.

        Returns:
            success: True if the internal QP was updated properly. Unlikely to be False.
        """

    def setStateLimits(
        self,
        x_min: typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"],
        x_max: typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"],
    ) -> bool:
        """
        Set state saturation limits.

        This function can only be called if `saturate_states` is enabled. Must be
        called prior to `initializeSolver()`, but can also be called after if limits
        change between solves.

        Args:
            x_min: Lower bound on state vector.
            x_max: Upper bound on state vector.

        Returns:
            success: True if the internal QP was updated properly. Unlikely to be False.
        """

    @typing.overload
    def setStateWeights(
        self, Q_diag: typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"]
    ) -> None:
        """
        Set only the state weights for the cost function.

        Args:
            Q_diag: Vector for diagonal of state weight matrix (non-negative).
        """

    @typing.overload
    def setStateWeights(
        self,
        Q_diag: typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"],
        Qf_diag: typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"],
    ) -> None:
        """
        Set only the state and terminal state weights for the cost function.

        Args:
            Q_diag: Vector for diagonal of state weight matrix (non-negative).
            Qf_diag: Vector for diagonal of terminal state weight matrix (non-negative).
        """

    @typing.overload
    def setWeights(
        self,
        Q_diag: typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"],
        R_diag: typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"],
    ) -> None:
        """
        Set state and input weights for the cost function.

        This method can only be called if `use_input_cost` is enabled.

        Args:
            Q_diag: Vector for diagonal of state weight matrix (non-negative).
            R_diag: Vector for diagonal of input weight matrix (non-negative).
        """

    @typing.overload
    def setWeights(
        self,
        Q_diag: typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"],
        Qf_diag: typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"],
        R_diag: typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"],
    ) -> None:
        """
        Set state, terminal state, and input weights for the cost function.

        This method can only be called if `use_input_cost` is enabled.

        Args:
            Q_diag: Vector for diagonal of state weight matrix (non-negative).
            Qf_diag: Vector for diagonal of terminal state weight matrix (non-negative).
            R_diag: Vector for diagonal of input weight matrix (non-negative).
        """

    def solve(
        self, x0: typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"]
    ) -> SolveStatus:
        """
        Solve optimization problem for the given initial state.

        Must have previously called initializeSolver() prior to calling this. Call
        getNextInput(), getInputControlPoints(), getInputTrajectory(), or
        getPredictedStateTrajectory() after calling this function to access the results.

        Args:
            x0: Initial (current) state vector.

        Returns:
            solve_status: Result indication. Generally expected to be `Success` unless
                the solver has not been initialized, then `NotInitialized`. Verify your
                problem setup and consult OSQP documentation for any other value.
        """

    @property
    def horizon_steps(self) -> int: ...
    @property
    def input_dim(self) -> int: ...
    @property
    def num_control_points(self) -> int: ...
    @property
    def state_dim(self) -> int: ...

class MPCLogger:
    """

    High-performance binary logger for MPC data and metadata.

    Uses a "write-raw, pack-later" strategy to support high logging frequencies.
    Per-step data is temporarily stored in binary files and then packed into a
    single .npz file during finalization. Metadata is stored in a .yaml file.
    Everything in the YAML file is also contained in the NPZ file, but the YAML
    provides a quick and easy way to see parameters from a simulation.

    The logger is designed to be used within a simulation or control loop. It
    provides a convenience method to automatically extract and stride
    trajectories from an MPC object.

    """

    def __init__(
        self,
        mpc: MPCBase,
        save_dir: os.PathLike[str] | str | bytes,
        ts: float,
        prediction_stride: int = 1,
        log_control_points: bool = False,
        save_name: str = "log",
    ) -> None:
        """
        Construct an MPCLogger for a given MPC instance; the logger is linked to this
        single MPC instance.

        IMPORTANT: The logger does not own the MPC object; the MPC instance must outlive
        the logger.

        The logger automatically captures a snapshot of the MPC object's current
        parameters (dimensions, weights, limits, etc.) at construction. If these
        parameters change later, captureMPCSnapshot() should be called manually.

        The destructor calls finalize() if it has not been manually called.

        Args:
            mpc: The MPC object from which to log parameters.
            save_dir: Directory to save log files. Created if it doesn't exist.
            ts: Model propagation time step used to align predicted trajectories in time.
            prediction_stride: Factor to downsample predicted trajectories.
                - 0: Log only the current step (minimal mode).
                - 1: Log every step of the horizon.
                - K: Log every K-th step of the horizon.
                Note: The terminal state (T) is always included if prediction_stride > 0.
            log_control_points: If true, logs control points of the parameterized input
                trajectory instead of the evaluated dense input trajectory.
            save_name: Base name for the .npz output file (default: "log").
        """

    def addMetadata(self, key: str, value: typing.Any, precision: int = -1) -> None:
        """
        Add or overwrite custom metadata to be saved in both NPZ and YAML.

        User-added metadata is preserved in the order it was added and appears after the
        automatic MPC snapshot in the output files.

        Args:
            key: Unique identifier for the metadata entry.
            value: The value to store (int, float, string, 1D NDArray, list[float]).
            precision: Optional decimal precision for floating point output in YAML.
                -1 uses default precision.
        """

    def captureMPCSnapshot(self) -> None:
        """
        Manually capture a snapshot of an MPC object's current parameters.

        This is called automatically in the constructor, but can be re-called if
        weights or limits are updated during the simulation.
        """

    def finalize(self) -> None:
        """
        Pack all temporary binary data into the final .npz file and write the parameter
        YAML file.

        This operation involves file I/O and should be called after the simulation
        loop ends. Temporary files are deleted upon successful completion.
        """

    def logStep(
        self,
        t: float,
        x0: typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"],
        solve_time: float = -1.0,
    ) -> None:
        """
        Log a single step of data, automatically fetching trajectories and references
        from the provided MPC object with applied striding logic.

        Args:
            t: Current simulation time (time of solve).
            x0: Current state at time t (same as provided to solve() since MPCBase does
                not store this).
            solve_time: Optional user-calculated solve time (likely to include setup
                time). The solve time reported by OSQP is also logged separately.
        """

    def writeParamFile(
        self, filename: os.PathLike[str] | str | bytes = "params.yaml"
    ) -> None:
        """
        Write the internal metadata map to a YAML file.

        Args:
            filename: Output filename (should end with .yaml or .yml).
        """

class OSQPSettings:
    """

    OSQP solver settings.

    Most users will likely not need to modify the default settings, but they are
    exposed here for advanced use cases. See OSQP documentation for details on each
    setting.

    Note: `eps_abs` and `eps_rel` are the main tolerances for convergence.
    Tightening these can improve solution accuracy but may increase solve time.
    Defaults are usually sufficient for MPC applications, but can be adjusted if
    convergence is an issue. OSQP default is 1e-3 while affine_mpc defaults to 1e-6.

    Attributes:
        adaptive_rho: Enable adaptive ADMM step size.
        adaptive_rho_fraction: Fraction of primal and dual residuals to trigger rho update.
        adaptive_rho_interval: Minimum number of iterations between rho updates.
        adaptive_rho_tolerance: Absolute residual tolerance for adaptive rho updates.
        alpha: Relaxation parameter. Default for QP solvers is 1.6.
            =1: no relaxation (standard ADMM).
            >1: over-relaxation (can improve convergence in some cases).
            <1: under-relaxation (rarely useful).

            If iters is high, consider increasing to 1.6-1.8. If oscillations occur,
            consider decreasing to 1.4-1.5. Can also try 1.0 for standard ADMM if
            convergence is an issue.
        check_termination: Check termination conditions at each iteration.
        delta: Regularization parameter for linear system solver.
        eps_abs: Absolute tolerance for termination.
        eps_dual_inf: Dual infeasibility tolerance for termination.
        eps_prim_inf: Primal infeasibility tolerance for termination.
        eps_rel: Relative tolerance for termination.
        linsys_solver: Linear system solver to use (direct, indirect, or unknown).
        max_iter: Maximum number of iterations.
        polishing: Enable polishing of the solution, which performs an additional
            solve after convergence to polish the solution. This can improve
            accuracy near,constraint limits, but adds computation time. For example,
            if the solution is at input saturation, tolerances may cause the
            solution to be slightly outside the limits, and polishing can help
            ensure the final solution respects the limits more accurately.
            An alternative to polishing is to saturate the solution after solving.
        polish_refine_iter: Number of refinement iterations for polishing.
        rho: ADMM step size.
        scaled_termination: Check termination conditions on scaled residuals.
        scaling: Enable problem data scaling.
        sigma: ADMM regularization parameter.
        time_limit: Time limit for solver in seconds.
        verbose: Enable verbose output from OSQP for each solve, which may be useful
            for debugging. For general MPC usage, this should be false.
        warm_starting: Enable warm starting with previous solution.

    """

    class LinsysSolverType:
        """
        Members:

          DirectSolver

          IndirectSolver

          UnknownSolver
        """

        DirectSolver: typing.ClassVar[
            OSQPSettings.LinsysSolverType
        ]  # value = <LinsysSolverType.DirectSolver: 1>
        IndirectSolver: typing.ClassVar[
            OSQPSettings.LinsysSolverType
        ]  # value = <LinsysSolverType.IndirectSolver: 2>
        UnknownSolver: typing.ClassVar[
            OSQPSettings.LinsysSolverType
        ]  # value = <LinsysSolverType.UnknownSolver: 0>
        __members__: typing.ClassVar[
            dict[str, OSQPSettings.LinsysSolverType]
        ]  # value = {'DirectSolver': <LinsysSolverType.DirectSolver: 1>, 'IndirectSolver': <LinsysSolverType.IndirectSolver: 2>, 'UnknownSolver': <LinsysSolverType.UnknownSolver: 0>}
        def __eq__(self, other: typing.Any) -> bool: ...
        def __getstate__(self) -> int: ...
        def __hash__(self) -> int: ...
        def __index__(self) -> int: ...
        def __init__(self, value: int) -> None: ...
        def __int__(self) -> int: ...
        def __ne__(self, other: typing.Any) -> bool: ...
        def __repr__(self) -> str: ...
        def __setstate__(self, state: int) -> None: ...
        def __str__(self) -> str: ...
        @property
        def name(self) -> str: ...
        @property
        def value(self) -> int: ...

    linsys_solver: OSQPSettings.LinsysSolverType
    @staticmethod
    def fromOSQPDefaults() -> OSQPSettings:
        """
        Factory method to create OSQPSettings with OSQP default values.
        """

    def __init__(self) -> None:
        """
        Constructor sets affine_mpc recommended values.
        """

    @property
    def adaptive_rho(self) -> int: ...
    @adaptive_rho.setter
    def adaptive_rho(self, arg0: int) -> None: ...
    @property
    def adaptive_rho_fraction(self) -> float: ...
    @adaptive_rho_fraction.setter
    def adaptive_rho_fraction(self, arg0: float) -> None: ...
    @property
    def adaptive_rho_interval(self) -> int: ...
    @adaptive_rho_interval.setter
    def adaptive_rho_interval(self, arg0: int) -> None: ...
    @property
    def adaptive_rho_tolerance(self) -> float: ...
    @adaptive_rho_tolerance.setter
    def adaptive_rho_tolerance(self, arg0: float) -> None: ...
    @property
    def alpha(self) -> float: ...
    @alpha.setter
    def alpha(self, arg0: float) -> None: ...
    @property
    def check_termination(self) -> int: ...
    @check_termination.setter
    def check_termination(self, arg0: int) -> None: ...
    @property
    def delta(self) -> float: ...
    @delta.setter
    def delta(self, arg0: float) -> None: ...
    @property
    def eps_abs(self) -> float: ...
    @eps_abs.setter
    def eps_abs(self, arg0: float) -> None: ...
    @property
    def eps_dual_inf(self) -> float: ...
    @eps_dual_inf.setter
    def eps_dual_inf(self, arg0: float) -> None: ...
    @property
    def eps_prim_inf(self) -> float: ...
    @eps_prim_inf.setter
    def eps_prim_inf(self, arg0: float) -> None: ...
    @property
    def eps_rel(self) -> float: ...
    @eps_rel.setter
    def eps_rel(self, arg0: float) -> None: ...
    @property
    def max_iter(self) -> int: ...
    @max_iter.setter
    def max_iter(self, arg0: int) -> None: ...
    @property
    def polish_refine_iter(self) -> int: ...
    @polish_refine_iter.setter
    def polish_refine_iter(self, arg0: int) -> None: ...
    @property
    def polishing(self) -> int: ...
    @polishing.setter
    def polishing(self, arg0: int) -> None: ...
    @property
    def rho(self) -> float: ...
    @rho.setter
    def rho(self, arg0: float) -> None: ...
    @property
    def scaled_termination(self) -> int: ...
    @scaled_termination.setter
    def scaled_termination(self, arg0: int) -> None: ...
    @property
    def scaling(self) -> int: ...
    @scaling.setter
    def scaling(self, arg0: int) -> None: ...
    @property
    def sigma(self) -> float: ...
    @sigma.setter
    def sigma(self, arg0: float) -> None: ...
    @property
    def time_limit(self) -> float: ...
    @time_limit.setter
    def time_limit(self, arg0: float) -> None: ...
    @property
    def verbose(self) -> int: ...
    @verbose.setter
    def verbose(self, arg0: int) -> None: ...
    @property
    def warm_starting(self) -> int: ...
    @warm_starting.setter
    def warm_starting(self, arg0: int) -> None: ...

class Options:
    """

    Controls which optional features are enabled at MPC construction time.

    All fields default to false. These options are immutable after constructing an
    MPC instance.

    Attributes:
        use_input_cost: Enables input regularization term in the cost function
            ((uref_k - u_k)^T R (uref_k - u_k)).
        slew_initial_input: Slew-rate constraint on initial input
            (|u0 - u_prev| <= u0_slew).
        slew_control_points: Enables slew-rate constraints on parameterization
            control points (|v_{i+1} - v_i| <= control_point_slew).
        saturate_states: Enables state saturation constraints.
        saturate_input_trajectory: Enables saturation of each input in the
            trajectory rather than just the control points. Only applicable for
            parameterizations with degree > 1. This adds constraints to the
            optimiztion, but can allow control points to be outside of input limits
            while keeping inputs within limits.

    """

    saturate_input_trajectory: bool
    saturate_states: bool
    slew_control_points: bool
    slew_initial_input: bool
    use_input_cost: bool
    def __init__(
        self,
        use_input_cost: bool = False,
        slew_initial_input: bool = False,
        slew_control_points: bool = False,
        saturate_states: bool = False,
        saturate_input_trajectory: bool = False,
    ) -> None:
        """
        Construct an Options struct defining optional MPC configuration features.

        Args:
            use_input_cost: Enables input regularization term in the cost function
                ((uref_k - u_k)^T R (uref_k - u_k)).
            slew_initial_input: Slew-rate constraint on initial input
                (|u0 - u_prev| <= u0_slew).
            slew_control_points: Enables slew-rate constraints on parameterization
                control points (|v_{i+1} - v_i| <= control_point_slew).
            saturate_states: Enables state saturation constraints.
            saturate_input_trajectory: Enables saturation of each input in the
                trajectory rather than just the control points. Only applicable for
                parameterizations with degree > 1. This adds constraints to the
                optimiztion, but can allow control points to be outside of input limits
                while keeping inputs within limits.
        """

class Parameterization:
    """

    Encapsulates B-spline input trajectory parameterization for MPC.

    Provides constructors and factory methods for generating knot vectors and
    metadata to map control points to per-step input trajectories over the
    prediction horizon.

    The preferred usage is via the static factory methods for common
    parameterizations.

    Attributes:
        horizon_steps: Number of discrete steps in the MPC horizon.
        degree: Degree of B-spline polynomials.
        num_control_points: Number of B-spline control points.
        knots: Full knot vector of size num_control_points + degree + 1.

    Static Methods:
        makeUniformClampedKnots: Generate a knot vector that can be modified.
        moveBlocking: Factory method for move-blocking parameterization.
        linearInterp: Factory method for linear interpolation parameterization.
        bspline: Factory method for clamped B-spline parameterization.

    """

    @staticmethod
    @typing.overload
    def bspline(
        horizon_steps: int, degree: int, num_control_points: int
    ) -> Parameterization:
        """
        Factory method for uniform clamped B-spline parameterization.

        Args:
            horizon_steps: Number of discrete steps in the MPC horizon.
            degree: Degree of B-spline polynomials. Must be less than num_control_points.
            num_control_points: Number of B-spline control points. Cannot be greater
                than horizon_steps.

        returns:
            out: Parameterization instance with uniform clamped B-spline structure.
        """

    @staticmethod
    @typing.overload
    def bspline(
        horizon_steps: int,
        degree: int,
        active_knots: typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"],
    ) -> Parameterization:
        """
        Factory method for clamped B-spline parameterization with custom active knots.

        Args:
            horizon_steps: Number of discrete steps in the MPC horizon.
            degree: Degree of B-spline polynomials. Must be less than horizon_steps.
            active_knots: Vector of active knots (need at least deg+1).

        returns:
            out: Parameterization instance with clamped B-spline structure.
        """

    @staticmethod
    @typing.overload
    def linearInterp(horizon_steps: int, num_control_points: int) -> Parameterization:
        """
        Factory method for uniform linear interpolation parameterization.

        Args:
            horizon_steps: Number of discrete steps in the MPC horizon.
            num_control_points: Number of B-spline control points. Cannot be greater
                than horizon_steps.

        returns:
            out: Parameterization instance with uniform linear interpolation structure.
        """

    @staticmethod
    @typing.overload
    def linearInterp(
        horizon_steps: int,
        endpoints: typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"],
    ) -> Parameterization:
        """
        Factory method for linear interpolation parameterization with custom endpoints.

        Args:
            horizon_steps: Number of discrete steps in the MPC horizon.
            endpoints: Vector of linear segment endpoint locations. Must include 0 and
                horizon_steps - 1.

        returns:
            out: Parameterization instance with linear interpolation structure.
        """

    @staticmethod
    def makeUniformClampedKnots(
        horizon_steps: int, degree: int, num_control_points: int
    ) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"]:
        """
        Static method for generating a uniform clamped knot vector. Useful for
        generating a starting point that can be modified for custom parameterizations.

        Args:
            horizon_steps: Number of discrete steps in the MPC horizon.
            degree: Degree of B-spline polynomials. Must be less than num_control_points.
            num_control_points: Number of B-spline control points. Cannot be greater
                than horizon_steps.

        Returns:
            knots: The uniform clamped knot vector.
        """

    @staticmethod
    @typing.overload
    def moveBlocking(horizon_steps: int, num_control_points: int) -> Parameterization:
        """
        Factory method for uniform move-blocking parameterization.

        Args:
            horizon_steps: Number of discrete steps in the MPC horizon.
            num_control_points: Number of B-spline control points. Cannot be greater
                than horizon_steps.

        returns:
            out: Parameterization instance with uniform move-blocking structure.
        """

    @staticmethod
    @typing.overload
    def moveBlocking(
        horizon_steps: int,
        change_points: typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"],
    ) -> Parameterization:
        """
        Factory method for move-blocking parameterization with custom change points.

        Args:
            horizon_steps: Number of discrete steps in the MPC horizon.
            change_points: Vector of change point locations.

        returns:
            out: Parameterization instance with move-blocking structure.
        """

    @typing.overload
    def __init__(
        self, horizon_steps: int, degree: int, num_control_points: int
    ) -> None:
        """
        Direct constructor for uniform clamped B-spline parameterization.

        Args:
            horizon_steps: Number of discrete steps in the MPC horizon.
            degree: Degree of B-spline polynomials. Must be less than num_control_points.
            num_control_points: Number of B-spline control points. Cannot be greater
                than horizon_steps.
        """

    @typing.overload
    def __init__(
        self,
        horizon_steps: int,
        degree: int,
        knots: typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"],
    ) -> None:
        """
        Direct constructor for advanced use cases with custom knot vector
        (e.g. unclamped B-splines).

        Args:
            horizon_steps: Number of discrete steps in the MPC horizon.
            degree: Degree of B-spline polynomials. Must be less than horizon_steps.
            knots: Full knot vector with size in the range [2*(degree+1),
                horizon_steps+degree+1]. Must be non-decreasing. First knot must be 0
                and last knot must be horizon_steps-1.
        """

    def evaluate(
        self,
        input_dim: int,
        control_points: typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"],
    ) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"]:
        """
        Evalutate an input trajectory from provided control_points.

        Args:
            input_dim: Dimension of input vector.
            control_points (vector): Parameterized input trajectory as a vector of size
                input_dim*num_control_points.

        Returns:
            input_traj (vector): The evaluated input trajectory.
        """

    @property
    def degree(self) -> int: ...
    @property
    def horizon_steps(self) -> int: ...
    @property
    def knots(
        self,
    ) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"]: ...
    @property
    def num_control_points(self) -> int: ...

class SolveStatus:
    """
    Members:

      Success

      NotInitialized

      SolvedInaccurate

      PrimalInfeasible

      DualInfeasible

      MaxIterReached

      TimeLimitReached

      OtherFailure
    """

    DualInfeasible: typing.ClassVar[
        SolveStatus
    ]  # value = <SolveStatus.DualInfeasible: 4>
    MaxIterReached: typing.ClassVar[
        SolveStatus
    ]  # value = <SolveStatus.MaxIterReached: 5>
    NotInitialized: typing.ClassVar[
        SolveStatus
    ]  # value = <SolveStatus.NotInitialized: 1>
    OtherFailure: typing.ClassVar[SolveStatus]  # value = <SolveStatus.OtherFailure: 7>
    PrimalInfeasible: typing.ClassVar[
        SolveStatus
    ]  # value = <SolveStatus.PrimalInfeasible: 3>
    SolvedInaccurate: typing.ClassVar[
        SolveStatus
    ]  # value = <SolveStatus.SolvedInaccurate: 2>
    Success: typing.ClassVar[SolveStatus]  # value = <SolveStatus.Success: 0>
    TimeLimitReached: typing.ClassVar[
        SolveStatus
    ]  # value = <SolveStatus.TimeLimitReached: 6>
    __members__: typing.ClassVar[
        dict[str, SolveStatus]
    ]  # value = {'Success': <SolveStatus.Success: 0>, 'NotInitialized': <SolveStatus.NotInitialized: 1>, 'SolvedInaccurate': <SolveStatus.SolvedInaccurate: 2>, 'PrimalInfeasible': <SolveStatus.PrimalInfeasible: 3>, 'DualInfeasible': <SolveStatus.DualInfeasible: 4>, 'MaxIterReached': <SolveStatus.MaxIterReached: 5>, 'TimeLimitReached': <SolveStatus.TimeLimitReached: 6>, 'OtherFailure': <SolveStatus.OtherFailure: 7>}
    def __eq__(self, other: typing.Any) -> bool: ...
    def __getstate__(self) -> int: ...
    def __hash__(self) -> int: ...
    def __index__(self) -> int: ...
    def __init__(self, value: int) -> None: ...
    def __int__(self) -> int: ...
    def __ne__(self, other: typing.Any) -> bool: ...
    def __repr__(self) -> str: ...
    def __setstate__(self, state: int) -> None: ...
    def __str__(self) -> str: ...
    @property
    def name(self) -> str: ...
    @property
    def value(self) -> int: ...

class SparseMPC(MPCBase):
    """

    MPC formulation where the predicted state trajectory and parameterization
    control points are both optimization design variables (sparse QP).

    Retains state and input variables with model as an optimization constraint,
    exploiting sparsity for scalability. Preferred for longer horizons or larger
    state dimensions. Converts the MPC problem to QP form for OSQP, using input
    parameterization.

    """

    @typing.overload
    def __init__(
        self,
        state_dim: int,
        input_dim: int,
        param: Parameterization,
        opts: Options = Options(),
    ) -> None:
        """
        Construct SparseMPC with specified input parameterization and MPC
        configuration options.

        Args:
           state_dim: State vector dimension.
           input_dim: Input vector dimension.
           param: Input trajectory parameterization.
           opts: Optional MPC configuration features to enable.
        """

    @typing.overload
    def __init__(
        self,
        state_dim: int,
        input_dim: int,
        horizon_steps: int,
        opts: Options = Options(),
    ) -> None:
        """
        Construct SparseMPC with no parameterization (full input trajectory will be
        optimized) and MPC configuration options.

        Args:
           state_dim: State vector dimension.
           input_dim: Input vector dimension.
           horizon_steps: Number of discrete steps in the MPC horizon.
           opts: Optional MPC configuration features to enable.
        """
