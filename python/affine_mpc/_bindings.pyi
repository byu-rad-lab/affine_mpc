"""
Affine MPC module
"""

import collections.abc
import enum
import numpy
import numpy.typing
import os
import scipy.sparse
import typing

__all__: list[str] = [
    "CondensedMPC",
    "Discretizer",
    "MPCBase",
    "MPCLogger",
    "OSQPSettings",
    "Options",
    "Parameterization",
    "QPBlock",
    "QuadraticProgram",
    "SolveInfo",
    "SolveStatus",
    "SparseMPC",
]

class Options:
    """
    Controls which optional features are enabled at MPC construction time.

    All flags default to false (bounded_states to empty, every state, and the
    constraint counts to zero). These options are immutable after constructing an
    MPC instance.

    Attributes:
        use_input_cost: Enables input regularization term in the cost function
            ((uref_k - u_k)^T R (uref_k - u_k)).
        slew_initial_input: Slew-rate constraint on initial input
            (|u0 - u_prev| <= u0_slew).
        slew_control_points: Enables slew-rate constraints on parameterization
            control points (|v_{i+1} - v_i| <= control_point_slew). Requires a
            B-spline parameterization.
        saturate_states: Enables state saturation constraints.
        saturate_input_trajectory: Enables saturation of the sampled input
            trajectory rather than just the control points. Ignored for degree 0,
            where the control points are the sampled inputs. For degree 1 only the
            samples adjacent to each knot are constrained (at most
            2*num_control_points-2 rows), which is exact for non-integer knots. For
            degree > 1 every sample is constrained. Allows control points to be
            outside of input limits while keeping inputs within limits. A custom
            basis (Parameterization.fromBasis()) always constrains every sample.
        use_input_rate_cost: Enables a cost on the change of the sampled inputs
            ((u_k - u_{k-1})^T R_rate (u_k - u_{k-1}) with u_{-1} = u_prev).
        soft_state_bounds: Makes the state limits soft (requires
            saturate_states): x_min - s <= x_k <= x_max + s with one slack s >= 0
            per bounded state, penalized by l1^T s + s^T diag(l2) s.
        bounded_states: Indices of the states with limits (requires
            saturate_states). Empty (the default) bounds every state.
        num_general_constraints: Number p of general linear constraints
            h_min <= Hx x_k + Hu u_k <= h_max at every step k = 0, ..., T-1 (x_0 is
            the measured state). Zero (the default) disables them.
        num_terminal_constraints: Number of terminal constraints
            hf_min <= Hf x_T <= hf_max. Zero (the default) disables them.

    """

    saturate_input_trajectory: bool
    saturate_states: bool
    slew_control_points: bool
    slew_initial_input: bool
    soft_state_bounds: bool
    use_input_cost: bool
    use_input_rate_cost: bool
    def __init__(
        self,
        use_input_cost: bool = False,
        slew_initial_input: bool = False,
        slew_control_points: bool = False,
        saturate_states: bool = False,
        saturate_input_trajectory: bool = False,
        use_input_rate_cost: bool = False,
        soft_state_bounds: bool = False,
        bounded_states: collections.abc.Sequence[int] = [],
        num_general_constraints: int = 0,
        num_terminal_constraints: int = 0,
    ) -> None:
        """
        Construct an Options struct defining optional MPC configuration features.

        Args:
            use_input_cost: Enables input regularization term in the cost function
                ((uref_k - u_k)^T R (uref_k - u_k)).
            slew_initial_input: Slew-rate constraint on initial input
                (|u0 - u_prev| <= u0_slew).
            slew_control_points: Enables slew-rate constraints on parameterization
                control points (|v_{i+1} - v_i| <= control_point_slew). Requires a
                B-spline parameterization.
            saturate_states: Enables state saturation constraints.
            saturate_input_trajectory: Enables saturation of the sampled input
                trajectory rather than just the control points. Ignored for degree 0,
                where the control points are the sampled inputs. For degree 1 only the
                samples adjacent to each knot are constrained (at most
                2*num_control_points-2 rows), which is exact for non-integer knots. For
                degree > 1 every sample is constrained. Allows control points to be
                outside of input limits while keeping inputs within limits. A custom
                basis (Parameterization.fromBasis()) always constrains every sample.
            use_input_rate_cost: Enables a cost on the change of the sampled inputs
                ((u_k - u_{k-1})^T R_rate (u_k - u_{k-1}) with u_{-1} = u_prev).
            soft_state_bounds: Makes the state limits soft (requires
                saturate_states): x_min - s <= x_k <= x_max + s with one slack s >= 0
                per bounded state, penalized by l1^T s + s^T diag(l2) s.
            bounded_states: Indices of the states with limits (requires
                saturate_states). Empty (the default) bounds every state.
            num_general_constraints: Number p of general linear constraints
                h_min <= Hx x_k + Hu u_k <= h_max at every step k = 0, ..., T-1 (x_0 is
                the measured state). Zero (the default) disables them.
            num_terminal_constraints: Number of terminal constraints
                hf_min <= Hf x_T <= hf_max. Zero (the default) disables them.
        """

    def __repr__(self) -> str: ...
    def __str__(self) -> str: ...
    @property
    def bounded_states(self) -> list[int]: ...
    @bounded_states.setter
    def bounded_states(self, arg0: collections.abc.Sequence[int]) -> None: ...
    @property
    def num_general_constraints(self) -> int: ...
    @num_general_constraints.setter
    def num_general_constraints(self, arg0: int) -> None: ...
    @property
    def num_terminal_constraints(self) -> int: ...
    @num_terminal_constraints.setter
    def num_terminal_constraints(self, arg0: int) -> None: ...

class Parameterization:
    """
    Encapsulates B-spline input trajectory parameterization for MPC.

    Provides constructors and factory methods for generating knot vectors and
    metadata to map control points to per-step input trajectories over the
    prediction horizon. fromBasis() instead takes any sampled basis matrix (e.g.
    Laguerre functions), whose coefficients play the role of the control points.

    The preferred usage is via the static factory methods for common
    parameterizations.

    Attributes:
        horizon_steps: Number of discrete steps in the MPC horizon.
        degree: Degree of B-spline polynomials (-1 for a custom basis).
        num_control_points: Number of B-spline control points (basis coefficients
            for a custom basis).
        knots: Full knot vector of size num_control_points + degree + 1 (empty for
            a custom basis).
        type: Kind of basis (Parameterization.Type).

    Static Methods:
        makeClampedKnots: Generate a uniform or geometric knot vector to modify.
        makeUniformClampedKnots: Generate a knot vector that can be modified.
        moveBlocking: Factory method for move-blocking parameterization.
        linearInterp: Factory method for linear interpolation parameterization.
        bspline: Factory method for clamped B-spline parameterization.
        fromBasis: Factory method for a custom sampled basis.

    """

    class Type(enum.Enum):
        """
        Kind of basis. Some MPC features rely on B-spline properties (see fromBasis()).
        """

        BSpline: typing.ClassVar[Parameterization.Type]
        Custom: typing.ClassVar[Parameterization.Type]

    @staticmethod
    @typing.overload
    def bspline(
        horizon_steps: int,
        degree: int,
        num_control_points: int,
        knot_ratio: float = 1.0,
    ) -> Parameterization:
        """
        Factory method for clamped B-spline parameterization with uniform or
        geometrically spaced active knots.

        Args:
            horizon_steps: Number of discrete steps in the MPC horizon.
            degree: Degree of B-spline polynomials. Must be less than num_control_points.
            num_control_points: Number of B-spline control points. Cannot be greater
                than horizon_steps.
            knot_ratio: Ratio of consecutive knot interval lengths (positive): the
                intervals are proportional to 1, knot_ratio, knot_ratio^2, ... and
                span [0, horizon_steps - 1]. The default of 1 gives uniform knots. A
                ratio above 1 puts short intervals (more flexibility) at the start of
                the horizon and a ratio below 1 puts them at the end.

        Raises:
            ValueError: If the knot intervals are so short that the sampled basis
                does not have full column rank, or if any argument is invalid.

        returns:
            out: Parameterization instance with clamped B-spline structure.
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
    def fromBasis(
        horizon_steps: int,
        basis: typing.Annotated[
            numpy.typing.NDArray[numpy.float64], "[m, n]", "flags.f_contiguous"
        ],
    ) -> Parameterization:
        """
        Factory method for a custom basis given by its samples, e.g. Laguerre
        functions. The input at step k is sum_i basis[k, i] * control_point[i], where
        the control points are the basis coefficients.

        A custom basis is treated as dense (every coefficient can affect every sample:
        band_width = num_control_points, band_start = 0, band_weights = basis
        transposed), and type is Type.Custom, with degree = -1 and empty knots. Since
        the B-spline convex hull property does not hold, an MPC with a custom basis
        always bounds every sampled input (whether or not saturate_input_trajectory is
        set), rejects slew_control_points, and rejects setReferenceInput(); the input
        cost weights the coefficients.

        Args:
            horizon_steps: Number of discrete steps in the MPC horizon.
            basis: Basis sampled at k = 0, ..., horizon_steps-1, of shape
                (horizon_steps, num_control_points) (one column per coefficient), with
                finite entries and full column rank: its smallest singular value must
                exceed 1e-9 times its largest. Full rank does not make the QP well
                conditioned; orthonormalizing the columns (e.g. by numpy.linalg.qr)
                helps for bases such as slowly decaying Laguerre functions.

        Raises:
            ValueError: If the shape is wrong, an entry is not finite, or the basis
                does not have full column rank.

        returns:
            out: Parameterization instance with a custom basis.
        """

    @staticmethod
    @typing.overload
    def linearInterp(
        horizon_steps: int, num_control_points: int, knot_ratio: float = 1.0
    ) -> Parameterization:
        """
        Factory method for linear interpolation parameterization with uniform or
        geometrically spaced endpoints.

        Args:
            horizon_steps: Number of discrete steps in the MPC horizon.
            num_control_points: Number of B-spline control points. Cannot be greater
                than horizon_steps.
            knot_ratio: Ratio of consecutive knot interval lengths (positive): the
                intervals are proportional to 1, knot_ratio, knot_ratio^2, ... and
                span [0, horizon_steps - 1]. The default of 1 gives uniform knots. A
                ratio above 1 puts short intervals (more flexibility) at the start of
                the horizon and a ratio below 1 puts them at the end.

        Raises:
            ValueError: If the segments are so short that the sampled basis does not
                have full column rank, or if any argument is invalid.

        returns:
            out: Parameterization instance with linear interpolation structure.
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
    def makeClampedKnots(
        horizon_steps: int,
        degree: int,
        num_control_points: int,
        knot_ratio: float = 1.0,
    ) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"]:
        """
        Static method for generating a clamped knot vector with uniform or
        geometrically spaced active knots (the knots used by the factories). Useful for
        generating a starting point that can be modified for custom parameterizations.

        Args:
            horizon_steps: Number of discrete steps in the MPC horizon.
            degree: Degree of B-spline polynomials. Must be less than num_control_points.
            num_control_points: Number of B-spline control points. Cannot be greater
                than horizon_steps.
            knot_ratio: Ratio of consecutive knot interval lengths (positive): the
                intervals are proportional to 1, knot_ratio, knot_ratio^2, ... and
                span [0, horizon_steps - 1]. The default of 1 gives uniform knots. A
                ratio above 1 puts short intervals (more flexibility) at the start of
                the horizon and a ratio below 1 puts them at the end.

        Raises:
            ValueError: If the knot intervals are so short that the sampled basis
                does not have full column rank, or if any argument is invalid.

        Returns:
            knots: The clamped knot vector.
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
    def moveBlocking(
        horizon_steps: int, num_control_points: int, knot_ratio: float = 1.0
    ) -> Parameterization:
        """
        Factory method for move-blocking parameterization with uniform or
        geometrically spaced blocks.

        Args:
            horizon_steps: Number of discrete steps in the MPC horizon.
            num_control_points: Number of B-spline control points. Cannot be greater
                than horizon_steps.
            knot_ratio: Ratio of consecutive knot interval lengths (positive): the
                intervals are proportional to 1, knot_ratio, knot_ratio^2, ... and
                span [0, horizon_steps - 1]. The default of 1 gives uniform knots. A
                ratio above 1 puts short intervals (more flexibility) at the start of
                the horizon and a ratio below 1 puts them at the end.

        Raises:
            ValueError: If the blocks are so short that some block has no sample of
                its own (the sampled basis would not have full column rank), or if
                any argument is invalid.

        returns:
            out: Parameterization instance with move-blocking structure.
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
            change_points: Increasing start times of the blocks. The first must be 0
                and the last at most horizon_steps - 1. horizon_steps - 1 is appended
                to close the last block, so 0, 1, ..., horizon_steps - 1 gives one
                control point per step.

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
                horizon_steps+degree+1]. Must be non-decreasing. The first active
                knot (index degree) must be 0 and the last active knot (index
                len(knots)-degree-1) must be horizon_steps-1; for unclamped splines
                the knots outside them extend beyond [0, horizon_steps-1]. Knots
                among the first and last degree+1 are snapped to 0 and
                horizon_steps-1 when within 1e-9 * horizon_steps (round-off). The
                basis sampled at k = 0, ..., horizon_steps-1 must have full column
                rank: every control point needs its own sample, in increasing order,
                at which its basis function is nonzero (Schoenberg-Whitney). Repeated
                knots are allowed, except an interior knot equal to the first active
                knot or, for degree > 0, the last.
        """

    def __repr__(self) -> str: ...
    def __str__(self) -> str: ...
    def evaluate(
        self,
        input_dim: int,
        control_points: typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"],
    ) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"]:
        """
        Evaluate an input trajectory from provided control_points.

        Args:
            input_dim: Dimension of input vector.
            control_points (vector): Parameterized input trajectory as a vector of size
                input_dim*num_control_points.

        Returns:
            input_traj (vector): The evaluated input trajectory.
        """

    @property
    def band_start(
        self,
    ) -> typing.Annotated[numpy.typing.NDArray[numpy.int32], "[m, 1]"]:
        """
        Index of the first control point that affects the input at each step k (size horizon_steps).
        """

    @property
    def band_weights(
        self,
    ) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, n]"]:
        """
        Basis weights in banded form (band_width x horizon_steps). The input at step k is sum_i band_weights[i, k] * control_point[band_start[k] + i].
        """

    @property
    def band_width(self) -> int:
        """
        Number of consecutive control points that can affect one sampled input (degree + 1 for B-splines).
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
    @property
    def type(self) -> Parameterization.Type:
        """
        Kind of basis.
        """

class Discretizer:
    """
    Discretizes a continuous-time affine model dx/dt = Ac x + Bc u + wc into
    x_next = Ad x + Bd u + wd, assuming the input is held constant over each time
    step (zero-order hold).

    The result is exact up to the truncation tolerance: Ad = exp(Ac dt),
    Bd = G Bc, and wd = G wc with G = integral_0^dt exp(Ac s) ds. The matrix
    exponential is computed with scaling and squaring. The last G is kept, so a new
    model offset can be discretized without recomputing the matrix exponential
    (see discretizeOffset()). Useful on its own for simulation and for
    updating the affine approximation of a nonlinear model.
    """

    def __init__(self, state_dim: int, input_dim: int) -> None:
        """
        Construct a Discretizer for models of the given dimensions.

        Args:
            state_dim: State vector dimension (positive).
            input_dim: Input vector dimension (positive).

        Raises:
            ValueError: If a dimension is not positive.
        """

    def __repr__(self) -> str: ...
    def discretize(
        self,
        Ac: typing.Annotated[
            numpy.typing.NDArray[numpy.float64], "[m, n]", "flags.f_contiguous"
        ],
        Bc: typing.Annotated[
            numpy.typing.NDArray[numpy.float64], "[m, n]", "flags.f_contiguous"
        ],
        wc: typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"],
        dt: float,
        tol: float = 1e-10,
    ) -> tuple[
        typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, n]"],
        typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, n]"],
        typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"],
    ]:
        """
        Discretize a continuous-time affine model.

        Args:
            Ac: Continuous-time state matrix (state_dim x state_dim).
            Bc: Continuous-time input matrix (state_dim x input_dim).
            wc: Continuous-time model offset (size state_dim).
            dt: Time step in seconds (positive).
            tol: Truncation tolerance for the Taylor series (positive). The actual
                error is typically two to three orders of magnitude smaller than tol.
                The default gives results close to machine precision; larger values
                (e.g., 1e-6) save a few matrix products.

        Returns:
            (Ad, Bd, wd): Discrete-time state matrix, input matrix, and model offset.

        Raises:
            ValueError: If a size is wrong, dt or tol is not positive, or Ac*dt is not
                finite.
        """

    def discretizeOffset(
        self, wc: typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"]
    ) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"]:
        """
        Discretize only a model offset with the state matrix and time step of the last
        discretize() call: wd = G wc. Costs one matrix-vector product, with no matrix
        exponential.

        Args:
            wc: Continuous-time model offset (size state_dim).

        Returns:
            wd: Discrete-time model offset.

        Raises:
            ValueError: If wc has the wrong size.
            RuntimeError: If discretize() has not been called.
        """

    def getExponentialIntegral(
        self,
    ) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, n]"]:
        """
        Get a copy of G = integral_0^dt exp(Ac s) ds from the last discretize() call
        (Bd = G Bc, wd = G wc).

        Raises:
            RuntimeError: If discretize() has not been called.
        """

    def getInputDim(self) -> int:
        """
        Get the input dimension.
        """

    def getStateDim(self) -> int:
        """
        Get the state dimension.
        """

    def hasDiscretized(self) -> bool:
        """
        Whether discretize() has been called.
        """

class OSQPSettings:
    """
    OSQP solver settings.

    Most users will likely not need to modify the default settings, but they are
    exposed here for advanced use cases. See OSQP documentation for details on each
    setting: https://osqp.org/docs/interfaces/solver_settings.html.

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
        cg_max_iter: Maximum iterations for conjugate gradient linear system solver.
        cg_precond: Preconditioner type for conjugate gradient solver.
        cg_tol_fraction: Conjugate gradient tolerance as fraction of initial residual.
        cg_tol_reduction: Number of conjugate gradient iterations before tol is halved.
        check_dualgap: Check duality gap at each iteration for termination.
        check_termination: Check termination interval.
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
        profiler_level: Level of detail for profiler annotations.
        rho: ADMM step size.
        rho_is_vec: Whether rho is a scalar or vector.
        scaled_termination: Check termination conditions on scaled residuals.
        scaling: Enable problem data scaling.
        sigma: ADMM regularization parameter.
        time_limit: Time limit for solver in seconds.
        verbose: Enable verbose output from OSQP for each solve, which may be useful
            for debugging. For general MPC usage, this should be false.
        warm_starting: Enable warm starting with previous solution.

    """

    class LinsysSolverType(enum.Enum):
        """
        Enum for linear system solver type.
        """

        DirectSolver: typing.ClassVar[OSQPSettings.LinsysSolverType]
        IndirectSolver: typing.ClassVar[OSQPSettings.LinsysSolverType]
        UnknownSolver: typing.ClassVar[OSQPSettings.LinsysSolverType]

    class PreconditionerType(enum.Enum):
        """
        Enum for preconditioner type.
        """

        DiagonalPreconditioner: typing.ClassVar[OSQPSettings.PreconditionerType]
        NoPreconditioner: typing.ClassVar[OSQPSettings.PreconditionerType]

    cg_precond: OSQPSettings.PreconditionerType
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

    def __repr__(self) -> str: ...
    def __str__(self) -> str: ...
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
    def cg_max_iter(self) -> int: ...
    @cg_max_iter.setter
    def cg_max_iter(self, arg0: int) -> None: ...
    @property
    def cg_tol_fraction(self) -> float: ...
    @cg_tol_fraction.setter
    def cg_tol_fraction(self, arg0: float) -> None: ...
    @property
    def cg_tol_reduction(self) -> int: ...
    @cg_tol_reduction.setter
    def cg_tol_reduction(self, arg0: int) -> None: ...
    @property
    def check_dualgap(self) -> int: ...
    @check_dualgap.setter
    def check_dualgap(self, arg0: int) -> None: ...
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
    def profiler_level(self) -> int: ...
    @profiler_level.setter
    def profiler_level(self, arg0: int) -> None: ...
    @property
    def rho(self) -> float: ...
    @rho.setter
    def rho(self, arg0: float) -> None: ...
    @property
    def rho_is_vec(self) -> int: ...
    @rho_is_vec.setter
    def rho_is_vec(self, arg0: int) -> None: ...
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

class SolveStatus(enum.Enum):
    """
    Enum for reporting the status of an MPC solve.
    """

    DualInfeasible: typing.ClassVar[SolveStatus]
    MaxIterReached: typing.ClassVar[SolveStatus]
    NotInitialized: typing.ClassVar[SolveStatus]
    OtherFailure: typing.ClassVar[SolveStatus]
    PrimalInfeasible: typing.ClassVar[SolveStatus]
    SolvedInaccurate: typing.ClassVar[SolveStatus]
    Success: typing.ClassVar[SolveStatus]
    TimeLimitReached: typing.ClassVar[SolveStatus]
    UpdateFailed: typing.ClassVar[SolveStatus]

class SolveInfo:
    """
    Solver diagnostics from the most recent solve. Obtain with
    MPCBase.getSolveInfo(). mpc_solve_time - qp_solve_time is the time spent
    outside the QP solver (QP assembly, data updates, and refactorization). Work
    done in MPCBase.prepare() is reported separately in prepare_time.
    """

    def __repr__(self) -> str: ...
    @property
    def iterations(self) -> int:
        """
        Number of solver iterations.
        """

    @property
    def mpc_solve_time(self) -> float:
        """
        Duration of the whole last solve() call after initialization, including QP assembly, data updates, refactorization, and the solver (seconds). Does not depend on OSQP profiling.
        """

    @property
    def objective(self) -> float:
        """
        QP objective value 1/2 z^T P z + q^T z at the solution. Omits constant terms of the MPC cost, so it is not the full MPC cost.
        """

    @property
    def prepare_time(self) -> float:
        """
        Total duration of the prepare() calls between the previous solve() and this one (seconds), zero if there were none. prepare_time + mpc_solve_time is the MPC's work for the step; mpc_solve_time alone is its latency.
        """

    @property
    def qp_solve_time(self) -> float:
        """
        Time the QP solver spent solving, including polishing (seconds). Zero when OSQP is built without profiling.
        """

    @property
    def status(self) -> SolveStatus:
        """
        Result of the solve. NotInitialized before initializeSolver(), and OtherFailure after initialization but before the first solve.
        """

class QuadraticProgram:
    """
    The QP an MPC solves,
    minimize 1/2 z^T P z + q^T z subject to l <= A z <= u,
    for analysis or for handing to another solver. Obtain with MPCBase.getQP() or
    MPCBase.assembleQP().

    P and A are scipy.sparse.csc_matrix objects holding the nonzero entries of the
    QP, in the same form for every formulation; P is full symmetric (both
    triangles). Reading P or A needs SciPy (pip install "affine-mpc[sparse]").
    The cost omits the constant terms of the MPC cost, so its value at the solution
    is not the full MPC cost. Equality rows have l = u, and unbounded sides are
    +-inf. The order of the variables z and of the rows of A is given by
    MPCBase.getQPVariableBlocks() and MPCBase.getQPConstraintBlocks().
    """

    def __init__(self) -> None:
        """
        Empty QP, to pass to MPCBase.assembleQP().
        """

    @property
    def A(self) -> scipy.sparse.csc_matrix:
        """
        Constraint matrix (scipy.sparse.csc_matrix).
        """

    @A.setter
    def A(self, arg0: scipy.sparse.csc_matrix) -> None: ...
    @property
    def P(self) -> scipy.sparse.csc_matrix:
        """
        Cost matrix (full symmetric, scipy.sparse.csc_matrix).
        """

    @P.setter
    def P(self, arg0: scipy.sparse.csc_matrix) -> None: ...
    @property
    def l(self) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"]:
        """
        Constraint lower bounds.
        """

    @l.setter
    def l(
        self, arg0: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[m, 1]"]
    ) -> None: ...
    @property
    def q(self) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"]:
        """
        Cost vector.
        """

    @q.setter
    def q(
        self, arg0: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[m, 1]"]
    ) -> None: ...
    @property
    def u(self) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"]:
        """
        Constraint upper bounds.
        """

    @u.setter
    def u(
        self, arg0: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[m, 1]"]
    ) -> None: ...

class QPBlock:
    """
    A named block of QP variables or constraint rows: indices offset, ...,
    offset + size - 1. Returned by MPCBase.getQPVariableBlocks() and
    MPCBase.getQPConstraintBlocks().
    """

    def __repr__(self) -> str: ...
    @property
    def name(self) -> str:
        """
        Block name, e.g. 'control_points' or 'input_bounds'.
        """

    @property
    def offset(self) -> int:
        """
        Index of the first variable or row.
        """

    @property
    def size(self) -> int:
        """
        Number of variables or rows.
        """

class MPCBase:
    """
    Base class. Use CondensedMPC or SparseMPC.
    """

    def __init__(
        self, state_dim: int, input_dim: int, param: Parameterization, opts: Options
    ) -> None:
        """
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
        """

    def __repr__(self) -> str: ...
    def __str__(self) -> str: ...
    def assembleQP(
        self,
        x0: typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"],
        qp: QuadraticProgram,
    ) -> None:
        """
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
        """

    def getBoundedStates(self) -> list[int]:
        """
        Get the indices of the bounded states (Options.bounded_states, or every state
        when it is empty; none without `saturate_states`).
        """

    def getGeneralConstraintInputMatrix(
        self,
    ) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, n]"]:
        """
        Get the input matrix Hu of the general constraints (zero until
        setGeneralConstraints()).

        Raises:
            RuntimeError: If `num_general_constraints` is zero.
        """

    def getGeneralConstraintMax(
        self,
    ) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"]:
        """
        Get the upper bounds of the general constraints (+inf until
        setGeneralConstraints()).

        Raises:
            RuntimeError: If `num_general_constraints` is zero.
        """

    def getGeneralConstraintMin(
        self,
    ) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"]:
        """
        Get the lower bounds of the general constraints (-inf until
        setGeneralConstraints()).

        Raises:
            RuntimeError: If `num_general_constraints` is zero.
        """

    def getGeneralConstraintStateMatrix(
        self,
    ) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, n]"]:
        """
        Get the state matrix Hx of the general constraints (zero until
        setGeneralConstraints()).

        Raises:
            RuntimeError: If `num_general_constraints` is zero.
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

        Raises:
            RuntimeError: If the solver has not been initialized.
            ValueError: If control_points has the wrong size.
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

        Raises:
            RuntimeError: If the solver has not been initialized.
        """

    def getInputMax(
        self,
    ) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"]:
        """
        Get the upper input limit (+inf until setInputLimits()).
        """

    def getInputMin(
        self,
    ) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"]:
        """
        Get the lower input limit (-inf until setInputLimits()).
        """

    def getInputRateWeights(
        self,
    ) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"]:
        """
        Get the diagonal of the input rate weight matrix R_rate (size input_dim).

        Raises:
            RuntimeError: If `use_input_rate_cost` is not enabled.
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

        Raises:
            RuntimeError: If the solver has not been initialized.
            ValueError: If u_traj has the wrong size.
        """

    @typing.overload
    def getInputTrajectory(
        self,
    ) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"]:
        """
        Get the full input trajectory from the previous solve.

        returns:
            u_traj (vector): The input trajectory.

        Raises:
            RuntimeError: If the solver has not been initialized.
        """

    def getInputWeights(
        self,
    ) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"]:
        """
        Get the diagonal of the input weight matrix R (size input_dim).

        Raises:
            RuntimeError: If `use_input_cost` is not enabled.
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

        Raises:
            RuntimeError: If the solver has not been initialized.
            ValueError: If u0 has the wrong size.
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

        Raises:
            RuntimeError: If the solver has not been initialized.
        """

    def getNumQPConstraints(self) -> int:
        """
        Get the number of QP constraint rows (the rows of A).
        """

    def getNumQPVariables(self) -> int:
        """
        Get the number of QP variables (the size of z).
        """

    def getOptions(self) -> Options:
        """
        Get a copy of the options enabled at construction.
        """

    def getParameterization(self) -> Parameterization:
        """
        Get a copy of the input trajectory parameterization.
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

        Raises:
            RuntimeError: If the solver has not been initialized.
            ValueError: If x_traj has the wrong size.
        """

    @typing.overload
    def getPredictedStateTrajectory(
        self,
    ) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"]:
        """
        Get the predicted state trajectory from the previous solve.

        returns:
            x_traj (vector): The predicted state trajectory.

        Raises:
            RuntimeError: If the solver has not been initialized.
        """

    def getPreviousInput(
        self,
    ) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"]:
        """
        Get the previous input used by the initial slew rate constraint and the input
        rate cost. Updated automatically after each solve that returns `Success` or
        `SolvedInaccurate`.

        Raises:
            RuntimeError: If neither `slew_initial_input` nor `use_input_rate_cost` is
                enabled.
        """

    def getQP(self) -> QuadraticProgram:
        """
        Analysis aid: get the QP as last written by solve(), prepare(), or assembleQP(),
        for analysis, debugging, or handing to another solver.

        Setters do not change it; to get the QP for the current settings and a given
        initial state without solving, use assembleQP(). Copies the QP, so not intended
        for the control loop.

        Returns:
            qp (QuadraticProgram): The QP; reading its P or A needs SciPy.
        """

    def getQPConstraintBlocks(self) -> list[QPBlock]:
        """
        Get the blocks of constraint rows of A, in order, listing only the blocks the
        options enable: "dynamics" (SparseMPC's model rows), "input_bounds",
        "initial_slew", "control_point_slew", "state_bounds" (soft: the upper rows, the
        lower rows, then the slack rows s >= 0), "general_constraints" (rows per step,
        steps in order), and "terminal_constraints". Together they cover every row.
        For analysis; allocates the list, so not intended for the control loop.

        Returns:
            blocks (list[QPBlock]): The constraint row blocks in order.
        """

    def getQPCostMatrix(
        self,
    ) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, n]"]:
        """
        Get a copy of the QP cost matrix P (the Hessian of the QP objective), for
        analysis and debugging.

        Reflects the matrix used by the last solve() or prepare(): setters do not change
        it, and model or weight changes are applied by the next prepare() or solve().

        SparseMPC stores P in sparse form, so each call also builds the dense matrix;
        not intended for the control loop.

        The solver reads only the upper triangle. For CondensedMPC this is the stored P,
        built by matrix products and so symmetric only up to round-off; for SparseMPC
        the lower triangle is mirrored from the upper one, so it is exactly symmetric.
        getQP() returns an exactly symmetric P for both.

        Returns:
            P (matrix): Copy of the full QP cost matrix.
        """

    def getQPVariableBlocks(self) -> list[QPBlock]:
        """
        Get the blocks of QP variables z, in order: "control_points" (input
        coefficients), then for SparseMPC "states" (x_1, ..., x_T, state_dim each), then
        with soft state bounds "state_slacks" (one per bounded state). Together they
        cover every variable. For analysis; allocates the list, so not intended for the
        control loop.

        Returns:
            blocks (list[QPBlock]): The variable blocks in order.
        """

    def getReferenceInputControlPoints(
        self,
    ) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"]:
        """
        Get the reference input control points as a stacked vector (size
        input_dim * num_control_points).

        Raises:
            RuntimeError: If `use_input_cost` is not enabled.
        """

    def getReferenceStateTrajectory(
        self,
    ) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"]:
        """
        Get the reference state trajectory as stacked states (size state_dim * horizon_steps).
        """

    def getSlewRate(
        self,
    ) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"]:
        """
        Get the control point slew rate (+inf until setSlewRate()).

        Raises:
            RuntimeError: If `slew_control_points` is not enabled.
        """

    def getSlewRateInitial(
        self,
    ) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"]:
        """
        Get the initial input slew rate (+inf until setSlewRateInitial()).

        Raises:
            RuntimeError: If `slew_initial_input` is not enabled.
        """

    def getSolveInfo(self) -> SolveInfo:
        """
        Get solver diagnostics from the last solve() (status, iterations, objective,
        and timing).

        After a solve() that returned `UpdateFailed`, the solver fields still describe
        the previous solve, while `mpc_solve_time` covers the failed call.
        `prepare_time` covers the prepare() calls between the previous solve() and
        the last one.

        Returns:
            info (SolveInfo): Default values (status `NotInitialized`) before
                initializeSolver().
        """

    def getStateBoundL1Penalties(
        self,
    ) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"]:
        """
        Get the L1 penalties of the soft state limits (zero until
        setStateBoundPenalties()).

        Raises:
            RuntimeError: If `soft_state_bounds` is not enabled.
        """

    def getStateBoundL2Penalties(
        self,
    ) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"]:
        """
        Get the L2 penalties of the soft state limits (zero until
        setStateBoundPenalties()).

        Raises:
            RuntimeError: If `soft_state_bounds` is not enabled.
        """

    def getStateMax(
        self,
    ) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"]:
        """
        Get the upper limits of the bounded states (+inf until setStateLimits()).

        Raises:
            RuntimeError: If `saturate_states` is not enabled.
        """

    def getStateMin(
        self,
    ) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"]:
        """
        Get the lower limits of the bounded states (-inf until setStateLimits()).

        Raises:
            RuntimeError: If `saturate_states` is not enabled.
        """

    def getStateWeights(
        self,
    ) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"]:
        """
        Get the diagonal of the state weight matrix Q (size state_dim).
        """

    def getTerminalConstraintMatrix(
        self,
    ) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, n]"]:
        """
        Get the state matrix Hf of the terminal constraints (zero until
        setTerminalConstraints()).

        Raises:
            RuntimeError: If `num_terminal_constraints` is zero.
        """

    def getTerminalConstraintMax(
        self,
    ) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"]:
        """
        Get the upper bounds of the terminal constraints (+inf until
        setTerminalConstraints()).

        Raises:
            RuntimeError: If `num_terminal_constraints` is zero.
        """

    def getTerminalConstraintMin(
        self,
    ) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"]:
        """
        Get the lower bounds of the terminal constraints (-inf until
        setTerminalConstraints()).

        Raises:
            RuntimeError: If `num_terminal_constraints` is zero.
        """

    def getTerminalStateWeights(
        self,
    ) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"]:
        """
        Get the diagonal of the terminal state weight matrix Q_f (size state_dim).
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

    def isSolverInitialized(self) -> bool:
        """
        Whether initializeSolver() has succeeded.
        """

    def isWithinSparsityPattern(self) -> bool:
        """
        Debugging aid: check whether the current QP matrices fit the sparsity pattern
        fixed by initializeSolver().

        Model, weight, or constraint matrix updates that add nonzeros outside that
        pattern are silently dropped by the solver, so the MPC solves a different
        problem than the one you set. Call this after prepare() or solve() (setters do
        not update the QP) to detect that. SparseMPC's pattern holds every entry its
        updates can write, so its QP always fits. It scans the QP matrices, so avoid
        calling it in time-critical loops.

        Returns:
            within_pattern (bool): True if the QP matrices fit the pattern, or if the
                solver is not initialized yet. False otherwise.
        """

    def prepare(self) -> SolveStatus:
        """
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

    def resetWarmStart(self) -> None:
        """
        Make the next solve() start from zero instead of warm starting from the previous
        solution (an OSQP cold start).

        Useful after a failed solve, whose iterates may have diverged and would slow
        down or prevent convergence of the next solve. Does not change the problem data,
        the previous input used by `slew_initial_input`, or the results returned by the
        solution getters. No-op before initializeSolver().
        """

    def setGeneralConstraints(
        self,
        Hx: typing.Annotated[
            numpy.typing.NDArray[numpy.float64], "[m, n]", "flags.f_contiguous"
        ],
        Hu: typing.Annotated[
            numpy.typing.NDArray[numpy.float64], "[m, n]", "flags.f_contiguous"
        ],
        h_min: typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"],
        h_max: typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"],
    ) -> None:
        """
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
        changed only when a value differs from the stored one. With CondensedMPC, as
        with the model, an entry of Hx or Hu that may become nonzero later must be
        nonzero when initializeSolver() is called, since that fixes the solver's
        sparsity pattern; SparseMPC keeps every entry of Hx and Hu in its pattern.

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
        """

    def setInputLimits(
        self,
        u_min: typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"],
        u_max: typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"],
    ) -> None:
        """
        Set input saturation limits.

        This function must be called prior to `initializeSolver()`, but can also be
        called after if limits change between solves.

        Args:
            u_min: Lower bound on input vector.
            u_max: Upper bound on input vector.

        Note:
            The QP is updated in the next prepare() or solve(), which report update
            failures.
        """

    def setInputRateWeights(
        self,
        R_rate_diag: typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"],
    ) -> None:
        """
        Set the weights of the input rate cost
        sum_k (u_k - u_{k-1})^T R_rate (u_k - u_{k-1}) over the sampled inputs,
        where u_{-1} is the previous input (see setPreviousInput()). Defaults to ones.

        This method can only be called if `use_input_rate_cost` is enabled. Unlike `R`
        (which weights the control points), the rate cost acts on the sampled inputs,
        so it penalizes the same input change equally for every basis.

        Args:
            R_rate_diag: Diagonal of the input rate weight matrix (size input_dim,
                non-negative).
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
        tol: float = 1e-10,
    ) -> None:
        """
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
    ) -> None:
        """
        Set the internal discrete-time model directly from a discrete model. The model
        being `x_next = Ax + Bu + w`.

        Args:
            Ad: Discrete-time state matrix A.
            Bd: Discrete-time input matrix B.
            wd: Discrete-time model offset w.

        Note:
            The QP is updated in the next prepare() or solve(), which report update
            failures.
        """

    def setModelOffsetContinuous2Discrete(
        self, wc: typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"]
    ) -> None:
        """
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
        """

    def setModelOffsetDiscrete(
        self, wd: typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"]
    ) -> None:
        """
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
        """

    def setPreviousInput(
        self, u_prev: typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"]
    ) -> None:
        """
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
        """

    def setReferenceInput(
        self, u_step: typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"]
    ) -> None:
        """
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
        """

    def setReferenceInputControlPoints(
        self,
        control_points: typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"],
    ) -> None:
        """
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
        """

    def setReferenceState(
        self, x_step: typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"]
    ) -> None:
        """
        Set reference state trajectory as a step command.

        Args:
            x_step: Reference state vector to use for entire trajectory.

        Note:
            The QP is updated in the next prepare() or solve(), which report update
            failures.
        """

    def setReferenceStateTrajectory(
        self, x_traj: typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"]
    ) -> None:
        """
        Set reference state trajectory.

        Args:
            x_traj (vector): Reference state trajectory as a vector of stacked states.
                Should have length of state_dim * horizon_steps. If you have a matrix
                where each row is the state at index k in the horizon, then you can pass
                in `x_traj.ravel()` to convert it to a vector.

        Note:
            The QP is updated in the next prepare() or solve(), which report update
            failures.
        """

    def setSlewRate(
        self,
        control_point_slew: typing.Annotated[
            numpy.typing.NDArray[numpy.float64], "[m, 1]"
        ],
    ) -> None:
        """
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
        """

    def setSlewRateInitial(
        self, u0_slew: typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"]
    ) -> None:
        """
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
        """

    def setStateBoundPenalties(
        self,
        l1: typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"],
        l2: typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"],
    ) -> None:
        """
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
        """

    def setStateLimits(
        self,
        x_min: typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"],
        x_max: typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"],
    ) -> None:
        """
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
        """

    @typing.overload
    def setStateWeights(
        self, Q_diag: typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"]
    ) -> None:
        """
        Set only the state weights for the cost function.

        The terminal state weights are also set to `Q_diag`.

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

    def setTerminalConstraints(
        self,
        Hf: typing.Annotated[
            numpy.typing.NDArray[numpy.float64], "[m, n]", "flags.f_contiguous"
        ],
        hf_min: typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"],
        hf_max: typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"],
    ) -> None:
        """
        Set the terminal constraints hf_min <= Hf x_T <= hf_max on the final predicted
        state.

        This function can only be called if `num_terminal_constraints` (p_T) is
        positive. Must be called prior to `initializeSolver()`, but can also be called
        after if the constraints change between solves. Pass `-inf` or `inf` for a side
        without a bound. As with setGeneralConstraints(), changing Hf rebuilds QP
        matrices and makes OSQP refactor, changing only the bounds updates QP vectors,
        and with CondensedMPC an entry of Hf that may become nonzero later must be
        nonzero when initializeSolver() is called (SparseMPC keeps every entry of Hf
        in its pattern).

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
        """

    @property
    def horizon_steps(self) -> int: ...
    @property
    def input_dim(self) -> int: ...
    @property
    def num_control_points(self) -> int: ...
    @property
    def state_dim(self) -> int: ...

class CondensedMPC(MPCBase):
    """
    MPC formulation where only parameterization control points are optimization
    design variables (condensed QP).

    Eliminates state variables analytically by implicitly wrapping the model into
    the cost function rather than as a constraint, resulting in a smaller dense
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

class MPCLogger:
    """
    High-performance binary logger for MPC data and metadata.

    Uses a "write-raw, pack-later" strategy to support high logging frequencies.
    Per-step data is always written to raw binary payload files during logging.
    Finalization then either keeps those raw payloads with recovery metadata,
    converts them to standalone `.npy` files, or packages them into a single `.npz`
    archive. Metadata is stored in a YAML file.

    The logger is designed to be used within a simulation or control loop. It
    provides a convenience method to automatically extract and stride trajectories
    from an MPC object.

    """

    class Mode(enum.Enum):
        """
        Controls how logged payloads are finalized after raw binary capture.
        """

        Npy: typing.ClassVar[MPCLogger.Mode]
        NpzCompressed: typing.ClassVar[MPCLogger.Mode]
        NpzUncompressed: typing.ClassVar[MPCLogger.Mode]
        RawRecoverable: typing.ClassVar[MPCLogger.Mode]

    def __init__(
        self,
        mpc: MPCBase,
        save_dir: os.PathLike[str] | str | bytes,
        ts: float,
        prediction_stride: int = 1,
        log_control_points: bool = False,
        save_name: str = "log",
        mode: MPCLogger.Mode = MPCLogger.Mode.NpzCompressed,
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
            save_name: Base name for the output artifact(s) (default: "log").
            mode: Finalization output mode. Raw payload files are always staged under
                `<save_name>_raw/` during logging.
                - RawRecoverable: saves data arrays as `*.npyh` + `*.bin` plus
                  `data_info.yaml` to `<save_name>_raw/`.
                - Npy: saves data arrays to `<save_name>_npy/*.npy`.
                - NpzUncompressed: saves all data into an uncompressed `<save_name>.npz`.
                - NpzCompressed: saves all data into a compressed `<save_name>.npz`.

                NPZ output is limited by ZIP32 size bounds of about 4 GiB. If NPZ
                finalization exceeds those limits, the logger falls back to NPY output
                and preserves raw recoverable payloads on failure.
        """

    def addMetadata(self, key: str, value: typing.Any, precision: int = -1) -> None:
        """
        Add or overwrite custom metadata to be saved in both the main output artifact
        and YAML metadata when supported.

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
        Finalize the logger output according to the configured logging mode.

        This operation involves file I/O and should be called after the simulation
        loop ends. Raw payloads are preserved on failure to support recovery.
        """

    def logStep(
        self,
        t: float,
        x0: typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"],
        loop_time: float = -1.0,
    ) -> None:
        """
        Log a single step of data, automatically fetching trajectories and references
        from the provided MPC object with applied striding logic.

        Args:
            t: Current simulation time (time of solve).
            x0: Current state at time t (same as provided to solve() since MPCBase does
                not store this).
            loop_time: Optional user-measured time (seconds), e.g. of the whole control
                loop iteration on hardware. Logged as `loop_time` (-1 when not
                provided). The `mpc_solve_time`, `qp_solve_time`, and `prepare_time`
                of the last solve (see SolveInfo) are logged automatically.

        Raises:
            RuntimeError: If the MPC solver has not been initialized.
            ValueError: If x0 does not have size state_dim.
        """

    def writeParamFile(
        self, filename: os.PathLike[str] | str | bytes = "params.yaml"
    ) -> None:
        """
        Write the internal metadata map to a YAML file.

        Args:
            filename: Output filename (should end with .yaml or .yml).
        """
