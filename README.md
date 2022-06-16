# affine_mpc

## Overview
`affine_mpc` is a C++ library that provides a convenient interface to the OSQP solver library in order to solve MPC problems that use an affine model. `affine_mpc_py` is a Pybind wrapper around the `affine_mpc` library in order to provide a Python interface.

## Dependencies
This library depends on [Eigen](https://gitlab.com/libeigen/eigen) (version >= 3.3.7) and [OSQP](https://github.com/osqp/osqp) (version 0.6.2). Eigen can be installed with `sudo apt install -y libeigen3-dev` and Ubuntu 20.04 will provide version 3.3.7, or it can be installed from source using the link above. OSQP should be installed from source using the link above.

## Building the Library
This library is designed to be built using CMake. There are 2 flags available to specify whether you would like to build the unit tests (`BUILD_TESTS`) and the `affine_mpc_py` library (`BUILD_PYTHON`). For example:

```shell
mkdir build && cd build
cmake -DBUILD_TESTS=OFF -DBUILD_PYTHON=ON ..
make
```

**NOTE:** It is recommended to build the `affine_mpc` library in `DEBUG` mode (`-DCMAKE_BUILD_TYPE=Debug`) while initially setting up your problem so that you can receive useful error statements regarding usable functions and size of function arguments. Once you have your problem running, then you can
build in `RELEASE` mode to gain some extra speed.

## MPC Problem
The following equations show the supported cost function and constraints within the `affine_mpc` library (the underlined portions with a red label are optional):

```math
\begin{equation}
J = \sum_{k=1}^{T+1} ||x_k - x_{k,des}||_Q + \underbrace{\sum_{k=0}^{p} ||u_k - u_{k,des}||_R}_{\textcolor{red}{\text{input cost}}}
\end{equation}
```

```math
\begin{align}
s.t. &\quad x_{k+1} = A x_k + B u_k + w \\
&\quad u_{min} \leq u_k \leq u_{max} \\
&\quad \underbrace{x_{min} \leq x_k \leq x_{max}}_{\textcolor{red}{\text{saturate states}}} \\
&\quad \underbrace{|u_{k+1} - u_k| \leq u_{slew}}_{\textcolor{red}{\text{slew rate}}}
\end{align}
```

where, $`J`$ is the total cost, $`T`$ is the horizon length, $`p`$ is the number of knot points used to parameterize the input trajectory over the horizon, $`n`$ is the number of states, $`m`$ is the number of inputs, $`A \in \mathbb{R}^{n \times n}`$, $`B \in \mathbb{R}^{n \times m}`$, $`w \in \mathbb{R}^n`$, $`x \in \mathbb{R}^n`$, $`u \in \mathbb{R}^m`$, and $`Q \in \mathbb{R}^{n \times n}`$ and $`R \in \mathbb{R}^{m \times m}`$ are positive semi-definite diagonal matrices.

**NOTE:** The norm in the cost function is a weighted 2-norm where $`||x||_M = x^\top M x`$.

<!--
Possible additions:
  - Terminal state weights (Q_final)
  - Terminal input weights (R_final)
  - Slew rate cost instead of constraint
  - State saturation cost?
  - Arbitrary constraint matrix addition
-->

## API
The C++ and Python APIs are almost identical, but I will try to highlight the differences here. Note that the C++ interface uses Eigen (fixed-size or dynamic matrices) while Python uses Numpy arrays: these types are not specified in the function declarations below to avoid being verbose.

### MPC Constructor
When you create an instance of any MPC class within the library, you must specify the number of states and inputs in your system, the horizon length and number of knot points you want to use in your prediction horizon, and the options you wish to use in your cost and constraint functions (shown [above](#mpc-problem) with red labels). Note that all of the cost and constraint options default to `false`. Once you specify all of these values in the constructor, those values can not change. All of the applicable values in the cost and constraint functions can be changed, but not the size and setup of the MPC problem. All MPC classes within this library inherit from the `MPCBase` interface class (which is not usable on its own as it has no solver - it is used to define a consistent interface with all of the MPC classes). All MPC classes in this library have the same constructor structure as `MPCBase`:

#### Function Declaration

```cpp
MPCBase(const int num_states, const int num_inputs,
        const int horizon_length, const int num_knot_points,
        const bool use_input_cost = false,
        const bool use_slew_rate = false,
        const bool saturate_states = false)
```

#### Usage
Here is an example of how to create and `ImplicitMPC` object:

<!-- TODO: link to Phil's paper on ImplicitMPC formulation -->

**C++**

```cpp
int num_states{2}, num_inputs{1}, horizon{10}, num_knot_points{3};
bool use_input_cost{true}, use_slew_rate{false}, saturate_states{true};
ImplicitMPC mpc{num_states, num_inputs, horizon, num_knot_points,
                use_input_cost, use_slew_rate, saturate_states};
```

**Python**

```python
mpc = ImplicitMPC(num_states=2, num_inputs=1,
                  horizon_length=10, num_knot_points=3,
                  use_input_cost=True, saturate_states=True)
```

### MPC Setup
After creating an MPC object with the format of the MPC problem you wish to use, you will need to specify all of the applicable parameters before initializing the solver (set the model, input saturation limits, slew rate, and state saturation limits). The state weights (Q) will default to identity while the input weights (R) will default to zero.

**Relevant Functions**

```cpp
void setModelDiscrete(Ad, Bd, wd);
void setModelContinuous2Discrete(Ac, Bc, wc, dt);
void setInputLimits(u_min, u_max);
void setSlewRate(u_slew); // if slew rate enabled
void setStateLimits(x_min, x_max); // if state saturation enabled
```

This means that if you linearize about equilibrium and have zeros that show up in your model that are not zero if you affinize about some other point, then you need to provide

### Initialize OSQP Solver
This library uses the OSQP solver for the optimization. After specifying the parameters from the previous section, the solver can be initialized. If you do not pass in `settings` then OSQP's default settings will be used.

**NOTE:** To learn more about the OSQP solver and its settings, visit the [OSQP website](https://osqp.org/docs/solver/index.html).

**Function Declaration**

```cpp
void initSolver(OSQPSettings* settings = nullptr);
```

**IMPORTANT NOTE:** The solver utilizes sparsity, meaning that the model used when the solver is initialized needs to have the least amount of sparsity possible for your system. The solver stores the structure and values of all non-zero elements when initialized and the structure can not change. This means that if it is possible for some elements of your model to be non-zero, then they need to be non-zero when the solver is initialized.

### Solve MPC
Once the solver is initialized, you need to specify weights and reference trajectories you wish to use:

**Relevant Functions**

```cpp
void setWeights(Q, R);
void setStateWeights(Q);
void setInputWeights(R);

void setDesiredState(x_step);
void setDesiredStateTrajectory(x_traj);

// if input cost enabled
void setDesiredInput(u_step);
void setDesiredInputTrajectory(u_traj);
```

Now you are ready to solve! You can either calculate the next input (first input from the prediction horizon) or calculate the entire input trajectory over the prediction horizon. They both do the exact same thing but if you only want the next input, it copies less numbers from the solver.

**Function Declarations C++**

```cpp
bool calcNextInput(x0, u);
bool calcInputTrajectory(x0, u_traj);
```

These two functions take in your current state and the input you are calcuating (passed by reference in order to set it) and returns a bool specifying whether the optimization solved correctly or not.

**Function Declarations Python**
Python provides the same interface as C++ for consistency (also if you don't want to allocate memory for the inputs every time), but you are also able to only pass in the current state while the inputs and bool specifying whether the optimization solved correctly are both always returned.

```python
calcNextInput( x0[, u] ) -> (u, solved)
calcInputTrajectory( x0[, u_traj] ) -> (u_traj, solved)
```

After solving, you can update any of the parameters before solving again (if you want to change your model, the weights, reference trajectories, etc.). Then you write a loop that will continuously pass in the current state and solve for inputs.

### Logging
An `MPCLogger` class is provided to log data for time, predicted state trajectory, optimized input trajectory, and reference state trajectory at each time step. This can be used as a debugging tool to visualize the entire solutions generated by the solver rather than just inputs that are actually applied to the system (usually only the first input calculated in the optimization).

#### Constructor

```cpp
MPCLogger(const MPCBase* const mpc, const std::string& save_location);
```

To create a logger object you must pass in a pointer to an existing MPC object along with a string for where you want the data to be stored (default is `"/tmp/mpc_data"`):

```cpp
MPCLogger logger{&mpc, "~/data/mpc"};
```

```python
logger = MPCLogger(mpc, '$HOME/data/mpc')
```

#### Log Data

```cpp
void logPreviousSolve(t, dt, x0, write_every=1);
```

Data can only be logged after calling a solve function on the MPC class the logger is tracking. To perform MPC you will likely have a loop of code that solves for an input to apply at each step. The loop can also update the desired state/input trajectory, change the weights, or update other pieces of the MPC problem, but the bare bones will look something like this:

```python
while t <= t_final:
  uk = mpc.calcNextInput(xk)
  xk = system.propagateDynamics(xk, uk) # or publish the command somehow
  logger.logPreviousSolve(t, dt, xk)
  t += dt
```

The logger creates 4 txt files of data within the `save_location` directory that contain all of the data. These files can be loaded in Python with `np.loadtxt('states.txt')` and each row of data represents a single time horizon (e.g., if there were 2 states and a horizon length of 3 then a row of data would contain 6 numbers for state 1 at the first time step, state 2 at the first time step, state 1 at the second time step, etc.). This pattern follows for `inputs.txt`, `time.txt`, and `ref_states.txt`.

The `write_every` variable is used to specify how frequently you wish to record data in a single time horizon (e.g., the default value of 1 will record all data, 2 will record every other time step, etc.).

#### Write Param File

```cpp
writeParamFile(const std::string& filename="params.yaml");
```

This function is used to write all of the parameters of the MPC problem setup to a file within the `save_location` directory. This way when you go looking back through multiple folders of data you can remember what params you used to generate plots (hopefully!). This function can be called as many times as you want, but you must specify a different file name if you want to keep multiple param files (using the same name will override the existing file). Perhaps this can be useful if you are tuning gains and want to know what they were at different points in time.

Also, the `MPCLogger` destructor is set to write a param file with the default name if you never call the function yourself. **NOTE: Python does not seem to call destructors in the correct order, so to avoid a runtime error after your script is finished you MUST have called this function manually at some point or else call `del logger` at the end of the script.** C++ does not have any issues with this.
