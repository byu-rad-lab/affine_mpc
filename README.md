# affine_mpc

## Overview

`affine_mpc` is a C++ library that provides a convenient interface to the OSQP solver library in order to solve Model Predictive Control (MPC) problems that use a discrete-time affine time-invariant model.

## License

This work is licensed with the BSD 3-clause license, see `LICENSE` file for details.

## Dependencies

Note that this project was developed using both Ubuntu 20.04 and 22.04 with the GCC compiler (version 11).

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
  - This project will locally clone and build cnpy automatically.

#### Optional:

- [GTest](https://google.github.io/googletest/) (1.11+) - Unit testing
  - Ubuntu: `sudo apt install libgtest-dev`
  - Arch: `sudo pacman -S --asdeps gtest`
  - Can install from [source](https://github.com/google/googletest)
  - A system install is recommended, but if not found then this project will locally clone and build GTest

## Building the Library

This library is designed to be built using CMake. There are 2 primary flags available:

- `AFFINE_MPC_BUILD_EXAMPLES` (specify whether to build the simulation examples)
- `AFFINE_MPC_BUILD_TESTS` (specify whether to build unit tests)

Run these in the top-level of the repo:

```sh
cmake -S . -B build -DAFFINE_MPC_BUILD_TESTS=OFF -DAFFINE_MPC_BUILD_EXAMPLES=ON
cmake --build build --config Release --parallel
```

**Note**: Building in `Debug` mode provides additional shape checks on Eigen variables, which may be useful in development.
Once developed though, `Release` mode will run _a lot_ faster.

## MPC Problem

The following equations show the supported cost function and constraints within the `affine_mpc` library (the underlined portions with a red label are optional):

### Cost Function

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
    &\quad \underbrace{|u_0 - u_{{-}1}| \leq u_{0,slew}}_{\textcolor{red}{\text{initial slew rate}}} \\
    &\quad \underbrace{|\nu_{i+1} - \nu_i| \leq \nu_{slew}}_{\textcolor{red}{\text{slew control points}}}
\end{align}
```

where,
$`x \in \mathbb{R}^n`$ is the state,
$`\bar{x} \in \mathbb{R}^n`$ is the reference state,
$`u \in \mathbb{R}^m`$ is the input,
$`\nu \in \mathbb{R}^m`$ is a control point used to parameterize the input trajectory,
$`\bar{\nu} \in \mathbb{R}^m`$ is a reference control point,
$`g`$ is the function to evaluate the parameterized input trajectory (e.g., using B-splines),
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
This [paper](https://arxiv.org/pdf/2001.04931.pdf) shows how most of the conversion is done;
this library does have some additional formulation options not described in the paper.
Note that Condensed MPC is what the paper calls Small Matrix Formulation,
and Sparse MPC is what the paper calls Large Matrix Formulation.

## Examples

An example doing MPC for a mass-spring-damper system is available in the `examples` folder.
You can run the C++ simulation executable `example_sim` and then use
`plot_sim_tracking.py` or `plot_sim_predictions.py`
to visualize the results (these scripts read the binary data logged by the `MPCLogger`).

## API

This library uses Eigen (both fixed size and dynamic size matrices work).

**Not all functionality is documented here - only the basics.**
You can see the interface to all available functions by looking at the header files.

### Step 1: Configuration

Before constructing an MPC object, you must configure the input trajectory parameterization and toggle optional features.

#### Parameterization

The `affine_mpc::Parameterization` class uses B-splines to reduce the number of optimization variables.
There are static factory methods for common setups:

```cpp
// Move-blocking (piecewise constant)
auto param = affine_mpc::Parameterization::moveBlocking(horizon_steps, num_control_points);

// Linear interpolation between control points
auto param = affine_mpc::Parameterization::linearInterp(horizon_steps, num_control_points);

// Clamped B-splines of arbitrary degree
auto param = affine_mpc::Parameterization::bspline(horizon_steps, degree, num_control_points);
```

#### Options

The `affine_mpc::Options` struct toggles various optional costs and constraints.
All options default to `false`.
These are immutable after the MPC instance is created.

```cpp
affine_mpc::Options opts;
opts.use_input_cost = true;           // Penalize deviation from input reference
opts.slew_initial_input = true;       // Limit change from previously applied input
opts.slew_control_points = true;      // Limit change between sequential control points
opts.saturate_states = true;          // Enable state boundaries
opts.saturate_input_trajectory = false; // Saturate dense trajectory (if degree > 1)
```

### Step 2: MPC Constructor

You can choose between `CondensedMPC` and `SparseMPC`.
All MPC classes inherit from the `MPCBase` interface.

```cpp
int state_dim = 4;
int input_dim = 2;

affine_mpc::CondensedMPC mpc(state_dim, input_dim, param, opts);
// OR
// affine_mpc::SparseMPC mpc(state_dim, input_dim, param, opts);
```

### Step 3: MPC Pre-Initialization Setup

**CRITICAL:** You must fully configure the MPC problem (model, limits, weights, reference trajectories) **BEFORE** initializing the solver.
The OSQP solver is a sparse solver that determines its internal memory structure during initialization based on non-zero elements.

If a matrix element might be non-zero in the future, it _must_ be non-zero when the solver is initialized.

```cpp
// 1. Set the Model
mpc.setModelDiscrete(Ad, Bd, wd);
// OR discretize from continuous
// mpc.setModelContinuous2Discrete(Ac, Bc, wc, dt);

// 2. Set Constraints (as enabled in Options)
mpc.setInputLimits(u_min, u_max);
if (opts.slew_initial_input) mpc.setSlewRateInitial(u0_slew);
if (opts.slew_control_points) mpc.setSlewRate(u_slew);
if (opts.saturate_states) mpc.setStateLimits(x_min, x_max);

// 3. Set Weights
if (opts.use_input_cost)
    mpc.setWeights(Q_diag, Qf_diag, R_diag);
else
    mpc.setStateWeights(Q_diag, Qf_diag);

// 4. Set Initial References
mpc.setReferenceState(x_ref);
// mpc.setReferenceStateTrajectory(x_traj);
if (opts.use_input_cost) {
    mpc.setReferenceInput(u_ref);
    // mpc.setReferenceParameterizedInputTrajectory(u_ref);
}
```

### Step 4: Initialize OSQP Solver

Once the problem is fully described, initialize the solver.

```cpp
// Uses recommended OSQP settings by default
if (!mpc.initializeSolver()) {
    std::cerr << "Failed to initialize solver!" << std::endl;
}
```

### Step 5: Solve MPC

In your control loop, provide the current state `xk` and call `solve()`.
You can update the model, weights, or references between solves.

```cpp
while (t <= t_final) {
  // Update any parameters (e.g., tracking a changing target)
  // mpc.setReferenceState(new_x_ref);
  // mpc.setModelDiscrete(new_Ad, new_Bd, new_wd); // Successive affinization

  // Solve
  affine_mpc::SolveStatus status = mpc.solve(xk);

  if (status != affine_mpc::SolveStatus::Success) {
    std::cerr << "Solver failed with status: " << status << std::endl;
    break;
  }

  // Retrieve the optimal inputs
  Eigen::VectorXd uk(input_dim);
  mpc.getNextInput(uk);

  // (Optional) Get full predicted trajectories
  // Eigen::VectorXd x_pred(state_dim * horizon_steps);
  // mpc.getPredictedStateTrajectory(x_pred);

  // Apply input and propagate simulation
  // (if MPC model does not match system, use actual model for propagation)
  mpc.propagateModel(xk, uk, xk);
  t += dt;
}
```

### Step 6: Logging (Optional)

The `MPCLogger` provides high-performance binary telemetry for simulation debugging.
It uses a "write-raw, pack-later" strategy to avoid bottlenecking high-frequency loops.

```cpp
// Initialize logger with the MPC instance, a save directory, and a simulation time step.
// The 'prediction_stride' allows downsampling prediction trajectories to save space.
int stride = 1;
affine_mpc::MPCLogger logger(mpc, "/tmp/mpc_data", dt, stride);

// Inside your loop, log data automatically (extracts trajectories from the mpc object)
logger.logStep(t, xk, mpc, user_solve_time);
```

The logger generates `log.npz` and `params.yaml` files.
The `.npz` binary file contains:

- `time`: `(N,)` simulation timestamps
- `states`: `(N, K, state_dim)` strided actual and predicted states.
- `ref_states`: `(N, K, state_dim)` strided reference states.
- `inputs`: `(N, K, input_dim)` strided actual and predicted inputs.
- `ref_inputs`: `(N, K, input_dim)` strided reference inputs.
- `solve_times`: `(N, 2)` user and OSQP-reported solve times.
- `meta_*`: Self-describing copies of all configuration parameters.

In Python, this is easily loaded for analysis:

```python
import numpy as np
data = np.load('/tmp/mpc_data/log.npz')
time = data["time"]
```

## Testing

Run tests using CTest from the build directory.

```shell
# Build and run all C++ unit tests
cmake -S . -B build -DAFFINE_MPC_BUILD_TESTS=ON
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

