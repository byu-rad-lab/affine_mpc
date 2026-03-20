# affine_mpc

## Overview

`affine_mpc` is a C++ library that provides a convenient interface to the OSQP solver library in order to solve MPC problems that use a discrete-time affine time-invariant model.

## License

This work is licensed with the BSD 3-clause license, see `LICENSE` file for details.

## Dependencies

Note that this project was developed using both Ubuntu 20.04 and 22.04 with the GCC
compiler (version 11).

#### Required:

- [Eigen](https://eigen.tuxfamily.org/dox/GettingStarted.html) (3.4+) - Matrix library
  - Ubuntu: `sudo apt install libeigen3-dev`
  - Arch: `sudo pacman -S --asdeps eigen`
  - Can install from [source](https://gitlab.com/libeigen/eigen)
  - A system install is recommended, but if not found then this project will locally clone and build Eigen
- [OSQP](https://osqp.org/docs/get_started/) (1.0+) - QP library
  - Arch: `sudo pacman -S --asdeps osqp`
  - Can install from [source](https://github.com/osqp/osqp)
  - This project will locally clone and build OSQP if it is not found on your system
- [cnpy](https://github.com/rogersce/cnpy.git) - Save NPY/NPZ files for logging
  - This project will locally clone and build cnpy

#### Optional:

- [GTest](https://google.github.io/googletest/) (1.11+) - Unit testing
  - Ubuntu: `sudo apt install libgtest-dev`
  - Arch: `sudo pacman -S --asdeps gtest`
  - Can install from [source](https://github.com/google/googletest)
  - A system install is recommended, but if not found then this project will locally clone and build GTest

## Building the Library

This library is designed to be built using CMake. There are 3 flags available:

- `affine_mpc_BUILD_EXAMPLE` (specify whether to build C++ example)
- `affine_mpc_BUILD_TESTS` (specify whether to build)

Run these in top-level of repo:

```sh
cmake -S . -B build -Daffine_mpc_BUILD_TESTS=OFF -Daffine_mpc_BUILD_BINDINGS=ON
cmake --build build --config Release --parallel
```

**Note**: Building in `Debug` mode provides additional shape checks on Eigen variables, which may be useful in
development.
Once developed though, `Release` mode will run _a lot_ faster.

## MPC Problem

The following equations show the supported cost function and constraints within the `affine_mpc` library (the underlined portions with a red label are optional):

### Cost Function

<!-- ```math -->
<!-- \begin{equation} -->
<!-- \argmin_{\nu_0,...,\nu_{p-1}} \quad \left\lVert \bar{x}_T - x_T \right\rVert^2_{Q_f} -->
<!--     + \sum_{k=1}^{T-1} \left\lVert \bar{x}_k - x_k \right\rVert^2_Q -->
<!--     + \underbrace{ -->
<!--         \sum_{i=0}^{p-1} \left\lVert \bar{\nu}_i - \nu_i \right\rVert^2_R -->
<!--       }_{\textcolor{red}{\text{input cost}}} -->
<!-- \end{equation} -->
<!-- ``` -->

```math
\begin{align}
\min
    &\quad J = \left\lVert \bar{x}_T - x_T \right\rVert^2_{Q_f}
    + \sum_{k=1}^{T-1} \left\lVert \bar{x}_k - x_k \right\rVert^2_Q
    + \underbrace{
        \sum_{i=0}^{p-1} \left\lVert \bar{\nu}_i - \nu_i \right\rVert^2_R
      }_{\textcolor{red}{\text{input cost}}} \\
w.r.t.
    &\quad \nu_0, \dots ,\nu_{p-1} \quad \underbrace{x_1, \dots, x_T}_{\textcolor{red}{\text{sparse only}}}\\
s.t.
    &\quad x_{k+1} = A x_k + B u_k + w \\
    &\quad u_k = g(\nu_0,...,\nu_{p-1}) \\
    &\quad u_{min} \leq \nu_k \leq u_{max} \quad \text{OR} \quad
        \underbrace{u_{min} \leq u_k \leq u_{max}}_{\textcolor{red}{\text{saturate input trajectory}}} \\
    &\quad \underbrace{x_{min} \leq x_k \leq x_{max}}_{\textcolor{red}{\text{saturate states}}} \\
    &\quad \underbrace{|u_{-1} - \nu_0| \leq \nu_{0,slew}}_{\textcolor{red}{\text{initial slew rate}}} \\
    &\quad \underbrace{|\nu_{i+1} - \nu_i| \leq \nu_{slew}}_{\textcolor{red}{\text{slew control points}}}
\end{align}
```

where,
$`x \in \mathbb{R}^n`$ is the state,
$`\bar{x} \in \mathbb{R}^n`$ is the reference state,
$`u \in \mathbb{R}^m`$ is the input,
$`\nu \in \mathbb{R}^m`$ is a control point used to parameterize the input trajectory,
$`\bar{\nu} \in \mathbb{R}^m`$ is a reference control point,
$`g`$ is the function to evaluate the parameterized input trajectory,
$T$ is the number of steps in the prediction horizon,
$`p`$ is the number of control points used to parameterize the input trajectory,
the discrete-time affine model is defined by
$`A \in \mathbb{R}^{n \times n}`$,
$`B \in \mathbb{R}^{n \times m}`$,
and $`w \in \mathbb{R}^n`$,
and $`Q \in \mathbb{R}^{n \times n}`$
and $`R \in \mathbb{R}^{m \times m}`$
are positive semi-definite diagonal weighting matrices.

**NOTE:** The norm in the cost function is a weighted 2-norm where
$`\left\lVert x \right\rVert^2_M = x^\top M x`$.

The MPC optimization problem must be converted to a QP optimization problem in order to use the OSQP solver.
This [paper](https://arxiv.org/pdf/2001.04931.pdf) shows how the conversion is done.
Note that Condensed MPC is what the paper calls Small Matrix Formulation,
and Sparse MPC is what the paper calls Large Matrix Formulation.

<!--
Possible additions:
  - Slew rate cost instead of constraint
  - State saturation cost?
  - Arbitrary constraint matrix addition
-->

## Examples

An example doing MPC for a mass-spring-damper system is available in the `examples` folder.
The `plot_sim_tracking.py` and `plot_sim_predictions.py` can be used to visualize the results,
one plots tracking only while the other also plots the prediction horizons for each solve.

## API

This library uses Eigen (both fixed size and dynamic size matrices work).

**Not all functionality is documented here - only the basics.**
You can see the interface to all available functions by looking at the header files.

### Step 1: MPC Constructor

When you create an instance of any MPC class within the library,
you must specify the dimention of the state and input vectors in your system,
the input trajectory parameterization,
and the options you wish to use in your cost and constraint functions
(shown [above](#mpc-problem) with red labels).
Note that all of the cost and constraint options default to `false`.
Once you specify all of these values in the constructor, those values can not change.
All of the applicable values in the cost and constraint functions can be changed,
but not the size and setup of the MPC problem.
All MPC classes within this library inherit from the `MPCBase` abstract interface class
(which is not usable on its own - it defines a consistent interface for all MPC classes).
All MPC classes in this library have the same constructor structure as `MPCBase`:

#### C++

```cpp
MPCBase(const int state_dim, const int input_dim,
        const int horizon_steps, const int num_control_points,
        const bool use_input_cost = false,
        const bool use_slew_rate = false,
        const bool saturate_states = false)
```

**Python**

```python
def __init__(self, state_dim: int, input_dim: int, horizon_steps: int,
             num_control_points: int, use_input_cost: bool=False,
             use_slew_rate: bool=False, saturate_states: bool=False)
```

### Step 2: MPC Pre-Initialization Setup

_AFTER_ creating an MPC object with the format of the MPC problem you wish to use, **you must specify all of the applicable parameters (set the model, input saturation limits, slew rate, and state saturation limits) _BEFORE_ initializing the solver.** The state weights (Q) will default to identity while the input weights (R) will default to zero.

**Relevant Member Functions**

#### C++

```cpp
void setModelDiscrete(const MatrixXd& Ad, const MatrixXd& Bd, const VectorXd& wd);
void setModelContinuous2Discrete(const MatrixXd& Ac, const MatrixXd& Bc,
                                 const VectorXd& wc, double dt);
void setInputLimits(const VectorXd& u_min, const VectorXd& u_max);
void setSlewRate(const VectorXd& u_slew); // if slew rate enabled
void setStateLimits(const VectorXd& x_min, const VectorXd& x_max); // if state saturation enabled
```

#### Python

```python
def setModelDiscrete(Ad: NDArray, Bd: NDArray, wd: NDArray) -> None:
def setModelContinuous2Discrete(Ac: NDArray, Bc: NDArray, wc: NDArray, dt: float) -> None:
def setInputLimits(u_min: NDArray, u_max: NDArray) -> None:
def setSlewRate(u_slew: NDArray) -> None: # if slew rate enabled
def setStateLimits(x_min: NDArray, x_max: NDArray) -> None: # if state saturation enabled
```

The OSQP solver is a sparse solver and it will only keep track of elements that are non-zero at the time of initialization. This means that the model used with the solver is initialized must be non-zero wherever it is possible to have non-zero values (if you are going to be changing the model at each time step - do not worry about this if you only ever use 1 model). For example, if a Jacobian of my A matrix is a rotation matrix then I need to make sure all 9 of those elements of A are non-zero rather than passing in an identity matrix if the rotation matrix will be updated after the solver is initialized.

**If you pass in a 0 somewhere that will not be a 0 later on in the code, the solver will not track the value and the model will not be what you expect it to be.** You might get lucky, but there is no safety check or guarantee that your code will work as expected if you are not careful when you initialize the solver.

### Step 3: Initialize OSQP Solver

This library uses the OSQP solver for the optimization. After specifying the parameters from the previous section, the solver can be initialized. If you do not pass in `solver_settings` then the default settings can be found in the `OSQPSolver` constructor, which modifies a couple of OSQP's default settings.

**NOTE:** To learn more about the OSQP solver and its settings, visit the [OSQP website](https://osqp.org/docs/solver/index.html).

#### C++

```cpp
bool initializeSolver(OSQPSettings* solver_settings = nullptr); // returns true if successful
```

#### Python

```python
def initializeSolver(solver_settings: OSQPSettings=None) -> bool: # returns true if successful
```

**REMINDER:** The solver utilizes sparsity, meaning that the model used when the solver is initialized needs to have the least amount of sparsity possible for your system. The solver stores the structure and values of all non-zero elements when initialized and the structure can not change. This means that if it is possible for some elements of your model to be non-zero, then they need to be non-zero when the solver is initialized.

### Step 4: Solve MPC

**You must have setup MPC and initialized the solver before you can solve!**

Once the solver is initialized, you need to specify weights and reference trajectories you wish to use:

#### Relevant Functions

**Note:** all of the following function arguents are 1D vectors/arrays.

#### C++

```cpp
void setWeights(const VectorXd& Q_diag, const VectorXd& R_diag);
void setStateWeights(const VectorXd& Q_diag); // will overwrite Qf assuming Qf=Q
void setStateWeightsTerminal(const VectorXd& Qf_diag); // call after setting state weights
void setInputWeights(const VectorXd& R_diag);

void setReferenceState(const VectorXd& x_step);
void setReferenceStateTrajectory(const VectorXd& x_traj);

// if input cost enabled
void setReferenceInput(const VectorXd& u_step);
void setReferenceParameterizedInputTrajectory(const VectorXd& u_traj_ctrl_pts);
```

#### Python

```python
def setWeights(Q_diag: NDArray, R_diag: NDArray) -> None:
def setStateWeights(Q_diag: NDArray) -> None: # will overwrite Qf assuming Qf=Q
def setStateWeightsTerminal(Qf_diag: NDArray) -> None: # call after setting state weights
def setInputWeights(R_diag: NDArray) -> None:

def setReferenceState(x_step: NDArray) -> None:
def setReferenceStateTrajectory(x_traj: NDArray) -> None:

# if input cost enabled
def setReferenceInput(u_step: NDArray) -> None:
def setReferenceParameterizedInputTrajectory(u_traj_ctrl_pts: NDArray) -> None:
```

#### Update parameters from pre-initialization setup

You can call any of the pre-initialization setup to update them before solving. If you want to successively linearize or affinize then you will need to update the model before each solve.

```cpp
setModelDiscrete(Ad, Bd, wd)
setModelContinuous2Discrete(Ac, Bc, wc, dt)
setInputLimits(u_min, u_max)
setSlewRate(u_slew) // if slew rate enabled
setStateLimits(x_min, x_max) // if state saturation enabled
```

Now you are ready to solve!

#### C++

```cpp
bool solve(const VectorXd& x0); // returns true if successful
```

#### Python

```python
def solve(x0: NDArray) -> bool: # returns true if successful
```

#### Get desired information from solve

You can get the next input to apply (the first input from the prediction horizon), the parameterized input trajectory, or the entire input trajectory over the prediction horizon. You can also get the predicted state trajectory (where MPC thinks the system will go).

#### C++

```cpp
void getNextInput(VectorXd& u0);
void getInputTrajectory(VectorXd& u_traj);
void getParameterizedInputTrajectory(VectorXd& u_traj_ctrl_pts);
void getPredictedStateTrajectory(VectorXd& x_traj);
```

#### Python

These functions allow you to call them like C++ where the return value is the function argument (avoids memory allocation and copying). They can also be called with no arguments, which will allocate memory and return the vector.

```python
def getNextInput([u0]) -> u0:
def getInputTrajectory([u_traj]) -> u_traj:
def getParameterizedInputTrajectory([u_traj_ctrl_pts]) -> u_traj_ctrl_pts:
def getPredictedStateTrajectory([x_traj]) -> x_traj:
```

After solving, you can update any of the parameters before solving again (if you want to change your model, the weights, reference trajectories, etc.). Then you write a loop that will continuously pass in the current state and solve for inputs.

### Step 5: Logging (Optional)

An `MPCLogger` class is provided to log data for time, predicted state trajectory, optimized input trajectory, and reference state trajectory at each time step. This can be used as a debugging tool to visualize the entire solutions generated by the solver rather than just inputs that are actually applied to the system (usually only the first input calculated in the optimization).

#### Constructor

#### C++

```cpp
MPCLogger(const MPCBase& mpc, const std::filesystem::path& save_location,
          double ts, int prediction_stride = 1, bool log_control_points = false);
// usage
MPCLogger logger{mpc, "~/data/mpc", 0.1, 2, false};
```

To create a logger object you must pass in a reference to an existing MPC object along with a string for where you want the data to be stored (default is `"/tmp/mpc_data"` and you can use `"~/"`, `"$HOME/"`, and `"${HOME}/"` at the beginning for your home directory - if the string does not start with one of these three strings or `"/"` then it is a path relative to the script).

You must also provide the time step `ts`. The `prediction_stride` parameter reduces file size by downsampling the predicted trajectories (e.g., `stride=2` logs every other step, but always includes the terminal state. `stride=0` logs only the current step). If `log_control_points` is true, the logger saves the raw QP control points instead of the evaluated input trajectory.

#### Logging

#### C++

```cpp
// Convenience: automatically gets trajectories from MPC
void logStep(double t, const Eigen::VectorXd& x0, const MPCBase& mpc, double solve_time = -1);
```

#### Python

```python
# Convenience: automatically gets trajectories from MPC
def logStep(t: float, x0: NDArray, mpc: MPCBase, solve_time: float = -1);
```

Data should only be logged after calling the solve function on the MPC class the logger is tracking. To perform MPC you will likely have a loop of code that solves for an input to apply at each step. The loop can also update the desired state/input trajectory, change the weights, or update other pieces of the MPC problem, but the bare bones will look something like this:

```python
while t <= t_final:
  now = time()
  # update any MPC parameters
  solved = mpc.solve(xk)
  solve_time = now - time()

  if not solved:
    # handle this how you want
    break

  logger.logStep(t, xk, mpc, solve_time)

  uk = mpc.getNextInput()
  xk = mpc.propagateModel(xk, uk)
  t += dt
```

The logger creates a `log.npz` binary file and a `params.yaml` metadata file within the `save_location` directory. The `.npz` file contains the following datasets:

- `time`: `(N,)` simulation timestamps
- `states`: `(N, K, state_dim)` where `states[:, 0, :]` is the actual state $x_0$, and subsequent indices are the strided predictions.
- `ref_states`: `(N, K, state_dim)` where `ref_states[:, 0, :]` is the reference at $t=1$, and subsequent indices perfectly align with the strided predictions.
- `inputs`: `(N, K, input_dim)` applied inputs and strided predictions (or `(N, num_control_points, input_dim)` if `log_control_points` is true).
- `ref_inputs`: `(N, K, input_dim)` applied inputs and strided references (or `(N, num_control_points, input_dim)` if `log_control_points` is true).
- `solve_times`: `(N, 2)` user and solver-reported times
- `meta_*`: Self-describing copies of all metadata, including `meta_t_pred` for perfectly aligning predicted trajectories in time.

These can be loaded in Python with `data = np.load('log.npz')`.

The `solve_time` parameter is optional. The logger records both this time and the solver-reported time to allow tracking of both setup and solve overhead.

#### Write Param File

#### C++

```cpp
void writeParamFile(const std::string& filename="params.yaml");
```

#### Python

```python
def writeParamFile(filename: str="params.yaml") -> None:
```

This function is used to write all of the parameters of the MPC problem setup to a file within the `save_location` directory. This way when you go looking back through multiple folders of data you can remember what params you used to generate plots (hopefully!). This function can be called as many times as you want, but you must specify a different file name if you want to keep multiple param files (using the same name will override the existing file). Perhaps this can be useful if you are tuning gains and want to know what they were at different points in time.

Also, the `MPCLogger` destructor is set to write a param file with the default name if you never call the function yourself. **NOTE: Python does not seem to call destructors in the correct order, so to avoid a runtime error after your script is finished you MUST have called this function manually at some point or else call `del logger` at the end of the script.** C++ does not have any issues with this.

## Testing

**Note:** The following commands are all written to be run inside of the top-level directory of the repository (assuming code is built in `build`).

#### Test C++ library only

```shell
./build/affine_mpc_UnitTests
```

#### Test Python bindings only

With `pytest`:

```shell
pytest
```

Without `pytest`:

```
python3 test/bindings/<testfile>.py
```

#### Test both the C++ library and its Python bindings

```shell
ctest --test-dir build
```

(if `BUILD_BINDINGS=OFF` then this will only run C++ tests):
