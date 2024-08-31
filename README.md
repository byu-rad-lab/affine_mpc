# affine_mpc

## Overview
`affine_mpc` is a C++ library that provides a convenient interface to the OSQP solver library in order to solve MPC problems that use an affine model. `affine_mpc_py` is a Pybind wrapper around the `affine_mpc` library in order to provide a Python interface.

## Dependencies
Note that this project was developed using both Ubuntu 20.04 and 22.04 with the GCC
compiler (version 11).

#### Required:
- [Eigen](https://eigen.tuxfamily.org/dox/GettingStarted.html)
  - Known to work with versions 3.3.7 and 3.4.0 - newer versions will likely also work
  - This is probably already installed on your system if you have ROS
  - Can install with `sudo apt install libeigen3-dev`
  - Can install from from [source](https://gitlab.com/libeigen/eigen)
- [OSQP](https://osqp.org/docs/get_started/)
  - Known to work with version 0.6.2
  - This project will locally clone and build OSQP for you if you do nothing
  - Can install to system from [source](https://github.com/osqp/osqp)

#### Optional:
- [GTest](https://google.github.io/googletest/) (required if you build the unit tests)
  - Known to work with version 1.11.0
  - This is probably already installed on your system if you have ROS
  - Can install to system with `sudo apt install libgtest-dev`
  - Can install to system from [source](https://github.com/google/googletest)
- [Pybind11](https://pybind11.readthedocs.io/en/stable/index.html) (required if you build Python bindings)
  - Known to work with versions 2.4.3 and 2.9.2
  - This project will locally clone and build Pybind11 version 2.9.2 for you if you do nothing
  - Can install to system with `sudo apt install pybind11-dev`, which will give you version 2.4.3 and you must specify the cmake variable `PYBIND11_VER=2.4.3` if you want to use this version
  - Can install to system from [source](https://github.com/pybind/pybind11)
- [NumPy](https://numpy.org/) (required if you want to use Python bindings)
  - Can install with 'pip install numpy'

## Building the Library
This library is designed to be built using CMake. There are 2 flags available to specify whether you would like to build the unit tests (`BUILD_TESTS`) and the `affine_mpc_py` library (`BUILD_PYTHON`). Additionally, you can set `-DPYBIND11_VER=2.4.3` to use the apt installed version of Pybind11 (on Ubuntu 20.04). For example:

```shell
mkdir build && cd build
cmake -DBUILD_TESTS=OFF -DBUILD_PYTHON=ON ..
make
```

**NOTE:** It is recommended to build the `affine_mpc` library in `DEBUG` mode (`-DCMAKE_BUILD_TYPE=Debug`) while initially setting up your problem so that you can receive useful error statements regarding usable functions and size of function arguments. Once you have your problem running, then you can
build in `RELEASE` mode to gain some extra speed.

#### Python Bindings
If you specified `-DBUILD_PYTHON=ON`, then the python bindings were built and placed into the `python/affine_mpc_py` package, which can be installed locally with pip:

```shell
pip install .
```
Then in a python file you can import the package:
```python
import affine_mpc_py as ampc
```

## MPC Problem
The following equations show the supported cost function and constraints within the `affine_mpc` library (the underlined portions with a red label are optional):

```math
\begin{equation}
% J = \sum_{k=1}^{T+1} ||x_k - x_{k,des}||_Q + \underbrace{\sum_{k=0}^{p} ||u_k - u_{k,des}||_R}_{\textcolor{red}{\text{input cost}}}
\text{argmin} \quad || \bar{x}_T - x_T ||^2_{Q_f} + \sum_{k=1}^{T-1} || \bar{x}_k - x_k ||^2_Q + \underbrace{\sum_{i=0}^{p-1} || \bar{\nu}_i - \nu_i ||^2_R}_{\textcolor{red}{\text{input cost}}}
\end{equation}
```

```math
\begin{align}
% \text{min} &\quad \sum_{k=1}^T ||x_k - x_{k,des}||^2_Q + \underbrace{\sum_{i=0}^{p-1} ||\nu_i - \nu_{i,des}||^2_R}_{\textcolor{red}{\text{input cost}}} \\
w.r.t &\quad \nu_0,...,\nu_{p-1} \\
s.t. &\quad x_{k+1} = A x_k + B u_k + w \\
&\quad u_k = g(\nu_0,...,\nu_{p-1}) \\
&\quad u_{min} \leq u_k \leq u_{max} \\
&\quad \underbrace{x_{min} \leq x_k \leq x_{max}}_{\textcolor{red}{\text{saturate states}}} \\
&\quad \underbrace{|\nu_{i+1} - \nu_i| \leq \nu_{slew}}_{\textcolor{red}{\text{slew rate}}}
\end{align}
```

where, $`x \in \mathbb{R}^n`$ is the state, $`\bar{x} \in \mathbb{R}^n`$ is the reference state, $`u \in \mathbb{R}^m`$ is the input, $`\nu \in \mathbb{R}^m`$ is a control point used to parameterize the input trajectory, $`\bar{\nu} \in \mathbb{R}^m`$ is a reference control point, $`g`$ is the function to evaluate the parameterized input trajectory, $`T`$ is the horizon length, $`p`$ is the number of control points used to parameterize the input trajectory, the discrete-time affine model is defined by $`A \in \mathbb{R}^{n \times n}`$, $`B \in \mathbb{R}^{n \times m}`$, and $`w \in \mathbb{R}^n`$, and $`Q \in \mathbb{R}^{n \times n}`$ and $`R \in \mathbb{R}^{m \times m}`$ are positive semi-definite diagonal weighting matrices.

**NOTE:** The norm in the cost function is a weighted 2-norm where $`||x||^2_M = x^\top M x`$.

The MPC optimization problem must be converted to a QP optimization problem in order to use the OSQP solver. This [paper](https://arxiv.org/pdf/2001.04931.pdf) shows how the conversion is done. Note that Implicit MPC is what the paper calls Small Matrix Formulation.

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

**Not all functionality is documented here - only the basics.** You can see the interface to all available functions by looking at the header files or using iPython on the Python library. Also, you can generate a stub file for the Python library with `stubgen -m affine_mpy_py -o stubs` (run at the location of the python library) if you want to use a stub file to enable autocompletion in an IDE like VS Code. More on [stubgen](https://manpages.ubuntu.com/manpages/focal/man1/stubgen.1.html).

### Step 1: MPC Constructor
When you create an instance of any MPC class within the library, you must specify the number of states and inputs in your system, the horizon length and number of control points you want to use in your prediction horizon, and the options you wish to use in your cost and constraint functions (shown [above](#mpc-problem) with red labels). Note that all of the cost and constraint options default to `false`. Once you specify all of these values in the constructor, those values can not change. All of the applicable values in the cost and constraint functions can be changed, but not the size and setup of the MPC problem. All MPC classes within this library inherit from the `MPCBase` interface class (which is not usable on its own as it has no usable solver - it is used to define a consistent interface with all of the MPC classes). All MPC classes in this library have the same constructor structure as `MPCBase`:

#### Constructor Function Declaration

```cpp
MPCBase(const int num_states, const int num_inputs,
        const int horizon_length, const int num_knot_points,
        const bool use_input_cost = false,
        const bool use_slew_rate = false,
        const bool saturate_states = false)
```

#### Usage
Here is an example of how to create and `ImplicitMPC` object:

**C++**

```cpp
int num_states{2}, num_inputs{1}, horizon{10}, num_control_points{3};
bool use_input_cost{true}, use_slew_rate{false}, saturate_states{true};
ImplicitMPC mpc{num_states, num_inputs, horizon, num_control_points,
                use_input_cost, use_slew_rate, saturate_states};
```

**Python**

```python
mpc = ImplicitMPC(num_states=2, num_inputs=1,
                  len_horizon=10, num_control_points=3,
                  use_input_cost=True, saturate_states=True)
```

### Step 2: MPC Pre-Initialization Setup
_AFTER_ creating an MPC object with the format of the MPC problem you wish to use, **you must specify all of the applicable parameters (set the model, input saturation limits, slew rate, and state saturation limits) _BEFORE_ initializing the solver.** The state weights (Q) will default to identity while the input weights (R) will default to zero.

**Relevant Functions**

```cpp
void setModelDiscrete(const MatrixXd& Ad, const MatrixXd& Bd, const VectorXd& wd);
void setModelContinuous2Discrete(const MatrixXd& Ac, const MatrixXd& Bc,
                                 const VectorXd& wc, double dt);
void setInputLimits(const VectorXd& u_min, const VectorXd& u_max);
void setSlewRate(const VectorXd& u_slew); // if slew rate enabled
void setStateLimits(const VectorXd& x_min, const VectorXd& x_max); // if state saturation enabled
```

The OSQP solver is a sparse solver and it will only keep track of elements that are non-zero at the time of initialization. This means that the model used with the solver is initialized must be non-zero wherever it is possible to have non-zero values (if you are going to be changing the model at each time step - do not worry about this if you only ever use 1 model). For example, if a Jacobian of my A matrix is a rotation matrix then I need to make sure all 9 of those elements of A are non-zero rather than passing in an identity matrix if the rotation matrix will be updated after the solver is initialized.

**If you pass in a 0 somewhere that will not be a 0 later on in the code, the solver will not track the value and the model will not be what you expect it to be.** You might get lucky, but there is no safety check or guarantee that your code will work as expected if you are not careful when you initialize the solver.

### Step 3: Initialize OSQP Solver
This library uses the OSQP solver for the optimization. After specifying the parameters from the previous section, the solver can be initialized. If you do not pass in `settings` then the default settings can be found in the `OSQPSolver` constructor, which modifies a couple of OSQP's default settings.

**NOTE:** To learn more about the OSQP solver and its settings, visit the [OSQP website](https://osqp.org/docs/solver/index.html).

**Function Declaration**

```cpp
bool initializeSolver(OSQPSettings* settings = nullptr); // returns true if successful
```

**IMPORTANT NOTE:** The solver utilizes sparsity, meaning that the model used when the solver is initialized needs to have the least amount of sparsity possible for your system. The solver stores the structure and values of all non-zero elements when initialized and the structure can not change. This means that if it is possible for some elements of your model to be non-zero, then they need to be non-zero when the solver is initialized.

### Step 4: Solve MPC

**You must have setup MPC and initialized the solver before you can solve!**

Once the solver is initialized, you need to specify weights and reference trajectories you wish to use:

#### Relevant Functions
**Note:** all of the following function arguents are 1D vectors.

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

#### Update parameters from pre-initialization setup
You can call any of the pre-initialization setup to update them before solving. If you want to successively linearize or affinize then you will need to update the model before each solve.

```cpp
void setModelDiscrete(const MatrixXd& Ad, const MatrixXd& Bd, const VectorXd& wd);
void setModelContinuous2Discrete(const MatrixXd& Ac, const MatrixXd& Bc,
                                 const VectorXd& wc, double dt);
void setInputLimits(const VectorXd& u_min, const VectorXd& u_max);
void setSlewRate(const VectorXd& u_slew); // if slew rate enabled
void setStateLimits(const VectorXd& x_min, const VectorXd& x_max); // if state saturation enabled
```

Now you are ready to solve!

```cpp
bool solve(const VectorXd& x0); // returns true if successful
```

#### Get desired information from solve
You can get the next input to apply (the first input from the prediction horizon), the parameterized input trajectory, or the entire input trajectory over the prediction horizon. You can also get the predicted state trajectory (where MPC thinks the system will go).

**Function Declarations C+**

```cpp
void getNextInput(VectorXd& u0);
void getInputTrajectory(VectorXd& u_traj);
void getParameterizedInputTrajectory(VectorXd& u_traj_ctrl_pts);
void getPredictedStateTrajectory(VectorXd& x_traj);
```

**Function Declarations Python**
These functions allow you to call them like C++ where the return value is the function argument (avoids memory allocation and copying). They can also be called with no arguments, which will allocate memory and return the vector.

```python
getNextInput([u0]) -> u0
getInputTrajectory([u_traj]) -> u_traj
getParameterizedInputTrajectory([u_traj_ctrl_pts]) -> u_traj_ctrl_pts
getPredictedStateTrajectory([x_traj]) -> x_traj
```

After solving, you can update any of the parameters before solving again (if you want to change your model, the weights, reference trajectories, etc.). Then you write a loop that will continuously pass in the current state and solve for inputs.

### Step 5: Logging (Optional)
An `MPCLogger` class is provided to log data for time, predicted state trajectory, optimized input trajectory, and reference state trajectory at each time step. This can be used as a debugging tool to visualize the entire solutions generated by the solver rather than just inputs that are actually applied to the system (usually only the first input calculated in the optimization).

#### Constructor

```cpp
MPCLogger(const MPCBase* const mpc, const std::string& save_location);
```

To create a logger object you must pass in a pointer to an existing MPC object along with a string for where you want the data to be stored (default is `"/tmp/mpc_data"` and you can use `"~/"`, `"$HOME/"`, and `"${HOME}/"` at the beginning for your home directory - if the string does not start with one of these three strings or `"/"` then it is a path relative to the script):

```cpp
MPCLogger logger{&mpc, "~/data/mpc"};
```

```python
logger = MPCLogger(mpc, '$HOME/data/mpc')
```

#### Log Data

```cpp
void logPreviousSolve(double t, double dt, const VectorXd& x0,
                      double solve_time=-1, int write_every=1);
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
  uk = mpc.getNextInput()
  logger.logPreviousSolve(t, dt, xk, solve_time)
  xk = system.propagateDynamics(xk, uk) # or publish the command somehow
  t += dt
```

The logger creates 4 txt files of data within the `save_location` directory that contain all of the data. These files can be loaded in Python with `np.loadtxt('states.txt')` and each row of data represents a single time horizon (e.g., if there were 2 states and a horizon length of 3 then a row of data would contain 6 numbers for state 1 at the first time step, state 2 at the first time step, state 1 at the second time step, etc.). This pattern follows for `inputs.txt`, `time.txt`, and `ref_states.txt`.

The `solve_time` function parameter is optional. The logger will record this time as well as the solve time reported from the solver on the same line. This is to allow the user to keep track of the time it takes to setup the MPC problem as well as the time it takes just to solve after it is setup.

The `write_every` variable is used to specify how frequently you wish to record data in a single time horizon (e.g., the default value of 1 will record all data, 2 will record every other time step, etc.).

#### Write Param File

```cpp
void writeParamFile(const std::string& filename="params.yaml");
```

This function is used to write all of the parameters of the MPC problem setup to a file within the `save_location` directory. This way when you go looking back through multiple folders of data you can remember what params you used to generate plots (hopefully!). This function can be called as many times as you want, but you must specify a different file name if you want to keep multiple param files (using the same name will override the existing file). Perhaps this can be useful if you are tuning gains and want to know what they were at different points in time.

Also, the `MPCLogger` destructor is set to write a param file with the default name if you never call the function yourself. **NOTE: Python does not seem to call destructors in the correct order, so to avoid a runtime error after your script is finished you MUST have called this function manually at some point or else call `del logger` at the end of the script.** C++ does not have any issues with this.

## Examples
Here are examples of a mass-spring-damper system in both C++ and Python.

### C++

```cpp
#include <Eigen/Core>
#include <iostream>

#include "affine_mpc/implicit_mpc.hpp"
#include "affine_mpc/mpc_logger.hpp"

namespace ampc = affine_mpc;


int main()
{
  const int n{2},m{1},T{10},p{10};
  const bool use_input_cost{true}, use_slew_rate{true};
  ampc::ImplicitMPC msd_mpc{n,m,T,p,use_input_cost,use_slew_rate};

  ampc::MPCLogger logger{&msd_mpc, "~/tmp/mpc_data"};

  Eigen::Matrix2d A;
  Eigen::Vector2d B, w;
  A << 0,1, -0.6,-0.1;
  B << 0, 0.2;
  w.setZero();
  double ts{0.1};
  msd_mpc.setModelContinuous2Discrete(A, B, w, ts);

  Eigen::Matrix<double,m,1> u_min{0}, u_max{3}, slew{1};
  msd_mpc.setInputLimits(u_min, u_max);
  msd_mpc.setSlewRate(slew);

  Eigen::Matrix<double,n,1> Q_diag{1,0.11};
  Eigen::Matrix<double,m,1> R_diag{.0001};
  msd_mpc.setWeights(Q_diag, R_diag);

  Eigen::Vector2d x_goal{1,0};
  Eigen::Matrix<double,m,1> u_goal{.0001};
  msd_mpc.setReferenceState(x_goal);
  msd_mpc.setReferenceInput(u_goal);

  msd_mpc.initializeSolver();

  Eigen::Vector2d xk;
  xk.setZero();

  Eigen::Matrix<double,m,1> uk;
  bool solved;
  double tf{5};
  while (double t{0}; t < tf; t += ts)
  {
    solved = msd_mpc.solve(xk);
    if (!solved)
      std::cout << "Did not solve :(" << endl;
    msd_mpc.getNextInput(uk);
    logger.logPreviousSolve(t, ts, xk);
    msd_mpc.propagateModel(xk, uk, xk);
  }

  return 0;
}
```

### Python

```python
import numpy as np
import affine_mpc_py as ampc


msd_mpc = ampc.ImplicitMPC(num_states=2, num_inputs=1,
                           horizon_length=10, num_knot_points=3,
                           use_input_cost=True, use_slew_rate=True)

logger = ampc.MPCLogger(msd_mpc, "~/tmp/mpc_data")

A = np.array([[0,1], [-0.6,-0.1]])
B = np.array([0,0.2])
w = np.zeros(2)
ts = 0.1
msd_mpc.setModelContinuous2Discrete(A, B, w, ts)

u_min = np.zeros(1)
u_max = np.ones(1) * 3
msd_mpc.setInputLimits(u_min, u_max)

slew = np.ones(1)
msd_mpc.setSlewRate(slew)

Q_diag = np.array([1,0.11])
R_diag = np.array([.0001])
msd_mpc.setWeights(Q_diag, R_diag)

x_goal = np.array([1.0,0])
u_goal = np.array([.0001])
msd_mpc.setReferenceState(x_goal)
msd_mpc.setReferenceInput(u_goal)

msd_mpc.initializeSolver()

xk = np.zeros(2)
t = 0.0
tf = 5.0
while t < tf:
  solved = msd_mpc.solve(xk)
  if not solved:
    print('Did not solve :(')
  uk = msd_mpc.calcNextInput()
  logger.logPreviousSolve(t, ts, xk)
  xk = msd_mpc.propagateModel(xk, uk)
  t += ts

logger.writeParamFile()
```

## Testing

To test only the C++ library (run inside of build directory):

```shell
./affine_mpc_UnitTests
```

To test only the Python bindings (run inside of main repository - requires `pytest`):

```shell
pytest
```

To test both the C++ library and its Python bindings (run inside of build directory):

```shell
ctest
```
