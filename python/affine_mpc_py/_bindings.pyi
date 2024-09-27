import flags
import numpy
from numpy.typing import NDArray
# from numpy.typing import flags
from typing import ClassVar, overload


class ImplicitMPC(MPCBase):
    def __init__(self, num_states: int, num_inputs: int, len_horizon: int,
                 num_control_points: int, use_input_cost: bool=False,
                 use_slew_rate: bool=False, saturate_states: bool=False) -> None:
        """
        :param num_states: Number of states in the system.
        :param num_inputs: Number of inputs in the system.
        :param len_horizon: Length of the prediction horizon.
        :param num_control_points: Number of control points used to parameterize the input trajectory.
        :param use_input_cost: Whether to penalize the input in the cost function with u.T @ R @ u.
        :param use_slew_rate: Whether to penalize the slew rate in the cost function.
        :param saturate_states: Whether to saturate the states.
        """
    @overload
    def getInputTrajectory(self, u_traj: NDArray[numpy.float64]) -> NDArray[numpy.float64]:
        """
        Get the optimized input trajectory: [u_0, ..., u_{T-1}].

        :param (m*T,) u_traj: Variable to store the optimized input trajectory.
        :return u_traj: (m*T,) The optimized input trajectory.
        """
    @overload
    def getInputTrajectory(self) -> NDArray[numpy.float64]:
        """
        Get the optimized input trajectory: [u_0, ..., u_{T-1}].

        :return u_traj: (m*T,) The optimized input trajectory.
        """
    @overload
    def getPredictedStateTrajectory(self, x_traj: NDArray[numpy.float64]) -> NDArray[numpy.float64]:
        """
        Get the predicted state trajectory: [x_1, ..., x_T].

        :param (n*T,) x_traj: Variable to store the predicted state trajectory.
        :return x_traj: (n*T,) The predicted state trajectory.
        """
    @overload
    def getPredictedStateTrajectory(self) -> NDArray[numpy.float64]:
        """
        Get the predicted state trajectory: [x_1, ..., x_T].

        :return x_traj: (n*T,) The predicted state trajectory.
        """
    def setInputLimits(self, u_min: NDArray[numpy.float64], u_max: NDArray[numpy.float64]) -> None:
        """
        Set the input limits: u_min <= u <= u_max.

        :param (m,) u_min: The minimum input values.
        :param (m,) u_max: The maximum input values.
        """
    def setSlewRate(self, u_slew: NDArray[numpy.float64]) -> None:
        """
        Set the slew rate: |u_k - u_{k-1}| <= u_slew. Can only call this function if use_slew_rate=True when constructed.

        :param (m,) u_slew: The slew rate.
        """
    def setStateLimits(self, x_min: NDArray[numpy.float64], x_max: NDArray[numpy.float64]) -> None:
        """
        Set the state limits: x_min <= x <= x_max. Can only call this function if saturate_states=True when constructed.

        :param (n,) x_min: The minimum state values.
        :param (n,) x_max: The maximum state values.
        """


class MPCBase:
    def __init__(self, num_states: int, num_inputs: int, len_horizon: int, num_control_points: int, use_input_cost: bool=False, use_slew_rate: bool=False, saturate_states: bool=False) -> None:
        """
        :param num_states: Number of states in the system.
        :param num_inputs: Number of inputs in the system.
        :param len_horizon: Length of the prediction horizon.
        :param num_control_points: Number of control points used to parameterize the input trajectory.
        :param use_input_cost: Whether to penalize the input in the cost function with u.T @ R @ u.
        :param use_slew_rate: Whether to penalize the slew rate in the cost function.
        :param saturate_states: Whether to saturate the states.
        """
    @overload
    def getInputTrajectory(self, u_traj: NDArray[numpy.float64]) -> NDArray[numpy.float64]:
        """
        Get the optimized input trajectory: [u_0, ..., u_{T-1}].

        :param (m*T,) u_traj: Variable to store the optimized input trajectory.
        :return u_traj: (m*T,) The optimized input trajectory.
        """
    @overload
    def getInputTrajectory(self) -> NDArray[numpy.float64]:
        """
        Get the optimized input trajectory: [u_0, ..., u_{T-1}].

        :return u_traj: (m*T,) The optimized input trajectory.
        """
    @overload
    def getNextInput(self, u0: NDArray[numpy.float64]) -> NDArray[numpy.float64]:
        """
        Get the next input to apply to the system, which is the first input from the optimized input trajectory.

        :param (m,) u0: Variable to store the optimal next input.
        :return u0: (m,) The optimal next input.
        """
    @overload
    def getNextInput(self) -> NDArray[numpy.float64]:
        """
        Get the next input to apply to the system, which is the first input from the optimized input trajectory.

        :return u0: (m,) The optimal next input.
        """
    @overload
    def getParameterizedInputTrajectory(self, u_traj_ctrl_pts: NDArray[numpy.float64]) -> NDArray[numpy.float64]:
        """
        Get the control points of the optimal parameterized input trajectory: [v_0, ..., v_{p-1}].

        :param (m*p,) u_traj_ctrl_pts: Variable to store the optimal parameterized input trajectory.
        :return u_traj_ctrl_pts: (m*p,) The optimal parameterized input trajectory.
        """
    @overload
    def getParameterizedInputTrajectory(self) -> NDArray[numpy.float64]:
        """
        Get the control points of the optimal parameterized input trajectory: [v_0, ..., v_{p-1}].

        :return u_traj_ctrl_pts: (m*p,) The optimal parameterized input trajectory.
        """
    @overload
    def getPredictedStateTrajectory(self, x_traj: NDArray[numpy.float64]) -> NDArray[numpy.float64]:
        """
        Get the predicted state trajectory: [x_1, ..., x_T].

        :param (n*T,) x_traj: Variable to store the predicted state trajectory.
        :return x_traj: (n*T,) The predicted state trajectory.
        """
    @overload
    def getPredictedStateTrajectory(self) -> NDArray[numpy.float64]:
        """
        Get the predicted state trajectory: [x_1, ..., x_T].

        :return x_traj: (n*T,) The predicted state trajectory.
        """
    def initializeSolver(self, solver_settings: OSQPSettings=None) -> bool:
        """
        Initialize the MPC solver with the given (or default) OSQP settings.

        :param OSQPSettings solver_settings: The solver settings.
        :return: Whether the solver was initialized successfully.
        """
    @overload
    def propagateModel(self, x: NDArray[numpy.float64], u: NDArray[numpy.float64], x_next: NDArray[numpy.float64]) -> NDArray[numpy.float64]:
        """
        Propagate the MPC model forward one time step.

        :param (n,) x: The current state.
        :param (m,) u: The input to apply over the time step.
        :param (n,) x_next: Variable to store the next state.
        :return x_next: (n,) The next state.
        """
    @overload
    def propagateModel(self, x: NDArray[numpy.float64], u: NDArray[numpy.float64]) -> NDArray[numpy.float64]:
        """
        Propagate the MPC model forward one time step.

        :param (n,) x: The current state.
        :param (m,) u: The input to apply over the time step.
        :return x_next: (n,) The next state.
        """
    def setInputLimits(self, u_min: NDArray[numpy.float64], u_max: NDArray[numpy.float64]) -> None:
        """
        Set the input limits: u_min <= u <= u_max.

        :param (m,) u_min: The minimum input values.
        :param (m,) u_max: The maximum input values.
        """
    def setInputWeights(self, R_diag: NDArray[numpy.float64]) -> None:
        """
        Set the input weights R, a diagonal matrix.

        :param (m,) R_diag: The input weights.
        """
    def setModelContinuous2Discrete(self, Ac: NDArray[numpy.float64], Bc: NDArray[numpy.float64], wc: NDArray[numpy.float64], dt: float, tol: float = ...) -> None:
        """
        Set the MPC model by discretizing the provided continuous-time model.

        :param (n,n) Ac: The continuous-time state matrix.
        :param (n,m) Bc: The continuous-time input matrix.
        :param (n,) wc: The continuous-time disturbance.
        :param float dt: The time step.
        :param float tol: The tolerance for discretization.
        """
    def setModelDiscrete(self, Ad: NDArray[numpy.float64], Bd: NDArray[numpy.float64], wd: NDArray[numpy.float64]) -> None:
        """
        Set the MPC model directly with the provided discrete-time model.

        :param (n,n) Ad: The discrete-time state matrix.
        :param (n,m) Bd: The discrete-time input matrix.
        :param (n,) wd: The discrete-time disturbance.
        """
    def setReferenceInput(self, u_step: NDArray[numpy.float64]) -> None:
        """
        Set the reference parameterized input trajectory to a step command. Can only call this function if use_input_cost=True when constructed.

        :param (m,) u_step: The reference input step command.
        """
    def setReferenceParameterizedInputTrajectory(self, u_traj_ctrl_pts: NDArray[numpy.float64]) -> None:
        """
        Set the reference parameterized input trajectory. Can only call this function if use_input_cost=True when constructed.

        :param (m*p,) u_traj_ctrl_pts: The reference parameterized input trajectory.
        """
    def setReferenceState(self, x_step: NDArray[numpy.float64]) -> None:
        """
        Set the reference state trajectory to a step command.

        :param (n,) x_step: The reference state step command.
        """
    def setReferenceStateTrajectory(self, x_traj: NDArray[numpy.float64]) -> None:
        """
        Set the reference state trajectory: [xref_1, ..., xref_T].

        :param (n*T,) x_traj: The reference state trajectory.
        """
    def setSlewRate(self, u_slew: NDArray[numpy.float64]) -> None:
        """
        Set the slew rate: |u_k - u_{k-1}| <= u_slew. Can only call this function if use_slew_rate=True when constructed.

        :param (m,) u_slew: The slew rate.
        """
    def setStateLimits(self, x_min: NDArray[numpy.float64], x_max: NDArray[numpy.float64]) -> None:
        """
        Set the state limits: x_min <= x <= x_max. Can only call this function if saturate_states=True when constructed.

        :param (n,) x_min: The minimum state values.
        :param (n,) x_max: The maximum state values.
        """
    def setStateWeights(self, Q_diag: NDArray[numpy.float64]) -> None:
        """
        Set the state weights Q, a diagonal matrix.

        :param (n,) Q_diag: The state weights.
        """
    def setStateWeightsTerminal(self, Qf_diag: NDArray[numpy.float64]) -> None:
        """
        Set the terminal state weights Qf, a diagonal matrix.

        :param (n,) Qf_diag: The terminal state weights.
        """
    def setWeights(self, Q_diag: NDArray[numpy.float64], R_diag: NDArray[numpy.float64]) -> None:
        """
        Set the state and input weights Q and R, both diagonal matrices.

        :param (n,) Q_diag: The state weights.
        :param (m,) R_diag: The input weights.
        """
    def solve(self, x0: NDArray[numpy.float64]) -> bool:
        """
        Solve the MPC problem with the given initial state. Can only call after initializeSolver.

        :param (n,) x0: The initial state.
        :return: Whether the problem was solved successfully.
        """
    @property
    def len_horizon(self) -> int:
        """
        :return: The length of the prediction horizon.
        """
    @property
    def num_ctrl_pts(self) -> int:
        """
        :return: The number of control points used to parameterize the input trajectory.
        """
    @property
    def num_inputs(self) -> int:
        """
        :return: The number of inputs in the system.
        """
    @property
    def num_states(self) -> int:
        """
        :return: The number of states in the system.
        """


class MPCLogger:
    def __init__(self, mpc: MPCBase, save_location: str='/tmp/mpc_data') -> None:
        """
        :param MPCBase mpc: The MPC object to log data from.
        :param str save_location: The directory to save the data.
        """
    def logPreviousSolve(self, t0: float, ts: float, x0: NDArray[numpy.float64], solve_time: float=-1, write_every: int=1) -> None:
        """
        Log the data from the previous MPC solve. Must be called after solve on the MPC object.

        :param float t0: The start time of the previous solve.
        :param float ts: The time step of the MPC prediction horizon from the previous solve.
        :param (n,) x0: The initial state of the previous solve.
        :param float solve_time: The time it took to solve the previous MPC problem.
        :param int write_every: How often to write the data to file. 1 means every time step, 2 means every other time step, etc.
        """
    def writeParamFile(self, filename: str = ...) -> None:
        """
        Write the parameter file to the save location to know what params were used to generate the data.

        :param str filename: The name of the parameter file.
        """


class OSQPSettings:
    class LinsysSolverType:
        __members__: ClassVar[dict] = ...  # read-only
        MKL_PARDISO_SOLVER: ClassVar[OSQPSettings.LinsysSolverType] = ...
        QDLDL_SOLVER: ClassVar[OSQPSettings.LinsysSolverType] = ...
        __entries: ClassVar[dict] = ...
        def __init__(self, value: int) -> None: ...
        def __eq__(self, other: object) -> bool: ...
        def __hash__(self) -> int: ...
        def __index__(self) -> int: ...
        def __int__(self) -> int: ...
        def __ne__(self, other: object) -> bool: ...
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
    linsys_solver: LinsysSolverType
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
    def __init__(self) -> None: ...
