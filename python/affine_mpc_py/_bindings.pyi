"""
Affine MPC module
"""

import numpy as np
from numpy.typing import NDArray
import typing

__all__ = ["BSplineMPC", "ImplicitMPC", "MPCBase", "MPCLogger", "OSQPSettings"]

class BSplineMPC(MPCBase):
    """
    BSpline MPC class
    """

    def __init__(
        self,
        num_states: int,
        num_inputs: int,
        len_horizon: int,
        num_control_points: int,
        spline_degree: int,
        knots: NDArray[np.float64] = ...,
        use_input_cost: bool = False,
        use_slew_rate: bool = False,
        saturate_states: bool = False,
    ) -> None:
        """
        Constructor.

        Args:
            num_states (int): Number of states in the system.
            num_inputs (int): Number of inputs in the system.
            len_horizon (int): Length of the prediction horizon.
            num_control_points (int): Number of control points used to parameterize
                the input trajectory.
            spline_degree (int): Degree of the polynomial segments of the input
                trajectory spline.
            knots (NDArray[np.float64]): Spline's knot point vector. Size must be equal
                to num_control_points - spline_degree + 1. Must be non-decreasing.
                If empty, uniform knots will be used.
            use_input_cost (bool): Whether to use the input cost. (uk.T @ R @ uk)
            use_slew_rate (bool): Whether to use a slew rate constraint.
                (|ctrl_next - ctrl_prev| <= slew)
            saturate_states (bool): Whether to saturate states. (x_min <= x_k <= x_max)
        """

    @typing.overload
    def getInputTrajectory(self, u_traj: NDArray[np.float64]) -> NDArray[np.float64]:
        """
        Get optimal trajectory of inputs from previous solve evaluated at each step in the prediction horizon.

        Args:
            u_traj (NDArray[numpy.float64]): Array in which to store the trajectory (same memory as output).
        Return:
            u_traj (NDArray[numpy.float64]): Optimized input trajectory (same memory as function argument)
        """

    @typing.overload
    def getInputTrajectory(self) -> NDArray[np.float64]:
        """
        Get optimal trajectory of inputs from previous solve evaluated at each step in the prediction horizon.

        Return:
            u_traj (NDArray[numpy.float64]): Optimized input trajectory (same memory as function argument)
        """

    @typing.overload
    def getPredictedStateTrajectory(
        self, x_traj: NDArray[np.float64]
    ) -> NDArray[np.float64]:
        """
        Get predicted state trajectory from previous solve
        """

    @typing.overload
    def getPredictedStateTrajectory(self) -> NDArray[np.float64]:
        """
        Get predicted state trajectory from previous solve
        """

    def setInputLimits(
        self, u_min: NDArray[np.float64], u_max: NDArray[np.float64]
    ) -> None:
        """
        Set input saturation limits
        """

    def setSlewRate(self, u_slew: NDArray[np.float64]) -> None:
        """
        Set slew rate constraint limits
        """

    def setStateLimits(
        self, x_min: NDArray[np.float64], x_max: NDArray[np.float64]
    ) -> None:
        """
        Set state saturation limits
        """

class ImplicitMPC(MPCBase):
    """
    Implicit MPC class
    """

    def __init__(
        self,
        num_states: int,
        num_inputs: int,
        len_horizon: int,
        num_control_points: int,
        use_input_cost: bool = False,
        use_slew_rate: bool = False,
        saturate_states: bool = False,
    ) -> None:
        """
        Constructor
        """

    @typing.overload
    def getInputTrajectory(self, u_traj: NDArray[np.float64]) -> NDArray[np.float64]:
        """
        Get optimal trajectory of inputs from previous solve
        """

    @typing.overload
    def getInputTrajectory(self) -> NDArray[np.float64]:
        """
        Get optimal trajectory of inputs from previous solve
        """

    @typing.overload
    def getPredictedStateTrajectory(
        self, x_traj: NDArray[np.float64]
    ) -> NDArray[np.float64]:
        """
        Get predicted state trajectory from previous solve
        """

    @typing.overload
    def getPredictedStateTrajectory(self) -> NDArray[np.float64]:
        """
        Get predicted state trajectory from previous solve
        """

    def setInputLimits(
        self, u_min: NDArray[np.float64], u_max: NDArray[np.float64]
    ) -> None:
        """
        Set input saturation limits
        """

    def setSlewRate(self, u_slew: NDArray[np.float64]) -> None:
        """
        Set slew rate constraint limits
        """

    def setStateLimits(
        self, x_min: NDArray[np.float64], x_max: NDArray[np.float64]
    ) -> None:
        """
        Set state saturation limits
        """

class MPCBase:
    """
    Not usable in Python!
    """

    def __init__(
        self,
        num_states: int,
        num_inputs: int,
        len_horizon: int,
        num_control_points: int,
        degree: int,
        knots: NDArray[np.float64] = ...,
        use_input_cost: bool = False,
        use_slew_rate: bool = False,
        saturate_states: bool = False,
    ) -> None:
        """
        Constructor
        """

    @typing.overload
    def getInputTrajectory(self, u_traj: NDArray[np.float64]) -> NDArray[np.float64]:
        """
        Get optimal trajectory of inputs from previous solve
        """

    @typing.overload
    def getInputTrajectory(self) -> NDArray[np.float64]:
        """
        Get optimal trajectory of inputs from previous solve
        """

    @typing.overload
    def getNextInput(self, u0: NDArray[np.float64]) -> NDArray[np.float64]:
        """
        Get next input to apply, i.e. first input in horizon from previous solve
        """

    @typing.overload
    def getNextInput(self) -> NDArray[np.float64]:
        """
        Get next input to apply, i.e. first input in horizon from previous solve
        """

    @typing.overload
    def getParameterizedInputTrajectory(
        self, u_traj_ctrl_pts: NDArray[np.float64]
    ) -> NDArray[np.float64]:
        """
        Get optimal parameterized trajectory of inputs from previous solve
        """

    @typing.overload
    def getParameterizedInputTrajectory(self) -> NDArray[np.float64]:
        """
        Get optimal parameterized trajectory of inputs from previous solve
        """

    @typing.overload
    def getPredictedStateTrajectory(
        self, x_traj: NDArray[np.float64]
    ) -> NDArray[np.float64]:
        """
        Get predicted state trajectory from previous solve
        """

    @typing.overload
    def getPredictedStateTrajectory(self) -> NDArray[np.float64]:
        """
        Get predicted state trajectory from previous solve
        """

    def initializeSolver(self, solver_settings: OSQPSettings | None = None) -> bool:
        """
        Initialize OSQP solver after configuring MPC setup
        """

    @typing.overload
    def propagateModel(
        self,
        x: NDArray[np.float64],
        u: NDArray[np.float64],
        x_next: NDArray[np.float64],
    ) -> NDArray[np.float64]:
        """
        Simulate internal model propagation step
        """

    @typing.overload
    def propagateModel(
        self, x: NDArray[np.float64], u: NDArray[np.float64]
    ) -> NDArray[np.float64]:
        """
        Simulate internal model propagation step
        """

    def setInputLimits(
        self, u_min: NDArray[np.float64], u_max: NDArray[np.float64]
    ) -> None:
        """
        Set input saturation limits
        """

    def setInputWeights(self, R_diag: NDArray[np.float64]) -> None:
        """
        Set state and input weights
        """

    def setModelContinuous2Discrete(
        self,
        Ac: NDArray[np.float64],
        Bc: NDArray[np.float64],
        wc: NDArray[np.float64],
        dt: float,
        tol: float = 1e-06,
    ) -> None:
        """
        Set internal model from discretized continuous model
        """

    def setModelDiscrete(
        self, Ad: NDArray[np.float64], Bd: NDArray[np.float64], wd: NDArray[np.float64]
    ) -> None:
        """
        Set internal model directly from discrete model
        """

    def setReferenceInput(self, u_step: NDArray[np.float64]) -> None:
        """
        Set desired input trajectory as a step command
        """

    def setReferenceParameterizedInputTrajectory(
        self, u_traj_ctrl_pts: NDArray[np.float64]
    ) -> None:
        """
        Set desired input trajectory
        """

    def setReferenceState(self, x_step: NDArray[np.float64]) -> None:
        """
        Set desired state trajectory as a step command
        """

    def setReferenceStateTrajectory(self, x_traj: NDArray[np.float64]) -> None:
        """
        Set desired state trajectory
        """

    def setSlewRate(self, u_slew: NDArray[np.float64]) -> None:
        """
        Set slew rate constraint limits
        """

    def setStateLimits(
        self, x_min: NDArray[np.float64], x_max: NDArray[np.float64]
    ) -> None:
        """
        Set state saturation limits
        """

    def setStateWeights(self, Q_diag: NDArray[np.float64]) -> None:
        """
        Set state weights
        """

    def setStateWeightsTerminal(self, Qf_diag: NDArray[np.float64]) -> None:
        """
        Set state weights
        """

    def setWeights(
        self, Q_diag: NDArray[np.float64], R_diag: NDArray[np.float64]
    ) -> None:
        """
        Set state and input weights
        """

    def solve(self, x0: NDArray[np.float64]) -> bool:
        """
        Solve optimization problem
        """

    @property
    def len_horizon(self) -> int: ...
    @property
    def num_ctrl_pts(self) -> int: ...
    @property
    def num_inputs(self) -> int: ...
    @property
    def num_states(self) -> int: ...

class MPCLogger:
    """
    MPC logger class
    """

    def __init__(self, mpc: MPCBase, save_location: str = "/tmp/mpc_data") -> None:
        """
        Constructor
        """

    def logPreviousSolve(
        self,
        t0: float,
        ts: float,
        x0: NDArray[np.float64],
        solve_time: float = -1,
        write_every: int = 1,
    ) -> None:
        """
        Log data from previous MPC solve
        """

    def writeParamFile(self, filename: str = "params.yaml") -> None:
        """
        Write current MPC params to a YAML file
        """

class OSQPSettings:
    """
    OSQP solver settings
    """

    class LinsysSolverType:
        """
        Members:

          QDLDL_SOLVER

          MKL_PARDISO_SOLVER
        """

        MKL_PARDISO_SOLVER: typing.ClassVar[
            OSQPSettings.LinsysSolverType
        ]  # value = <LinsysSolverType.MKL_PARDISO_SOLVER: 1>
        QDLDL_SOLVER: typing.ClassVar[
            OSQPSettings.LinsysSolverType
        ]  # value = <LinsysSolverType.QDLDL_SOLVER: 0>
        __members__: typing.ClassVar[
            dict[str, OSQPSettings.LinsysSolverType]
        ]  # value = {'QDLDL_SOLVER': <LinsysSolverType.QDLDL_SOLVER: 0>, 'MKL_PARDISO_SOLVER': <LinsysSolverType.MKL_PARDISO_SOLVER: 1>}
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

    adaptive_rho: int
    adaptive_rho_fraction: float
    adaptive_rho_interval: int
    adaptive_rho_tolerance: float
    alpha: float
    check_termination: int
    delta: float
    eps_abs: float
    eps_dual_inf: float
    eps_prim_inf: float
    eps_rel: float
    max_iter: int
    polish: int
    polish_refine_iter: int
    rho: float
    scaled_termination: int
    scaling: int
    sigma: float
    time_limit: float
    verbose: int
    warm_start: int
    def __init__(self) -> None:
        """
        Constructor sets all default values
        """

    @property
    def linsys_solver(self) -> OSQPSettings.LinsysSolverType: ...
    @linsys_solver.setter
    def linsys_solver(self, arg1: int) -> None: ...
