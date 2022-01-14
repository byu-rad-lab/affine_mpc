# affine_mpc

## Overview
`affine_mpc` is a C++ library that provides a convenient interface to the OSQP solver library in order to solve MPC problems that use an affine model. `affine_mpc_py` is a Pybind wrapper around the `affine_mpc` library in order to provide a Python interface.

## Dependencies
This library depends on [Eigen](https://gitlab.com/libeigen/eigen) (version >= 3.3.7) and [OSQP](https://github.com/osqp/osqp) (version 0.6.2). Eigen can be installed with `sudo apt install -y libeigen3-dev` and Ubuntu 20.04 will provide version 3.3.7, or it can be installed from source using the link above. OSQP should be installed from source using the link above.

## MPC Problem
The following equations show the supported cost function and constraints within the `affine_mpc` library (the underlined portions with a red label are optional):

$$
\begin{equation}
J = \sum_{k=1}^{T+1} ||x_k - x_{k,des}||_Q + \underbrace{\sum_{k=0}^{T} ||u_k - u_{k,des}||_R}_{\textcolor{red}{\text{input cost}}}
\end{equation}
$$

$$
\begin{align}
s.t. &\quad x_{k+1} = A x_k + B u_k \\
&\quad u_{min} \leq u_k \leq u_{max} \\
&\quad \underbrace{x_{min} \leq x_k \leq x_{max}}_{\textcolor{red}{\text{saturate states}}} \\
&\quad \underbrace{|u_{k+1} - u_k| \leq u_{slew}}_{\textcolor{red}{\text{slew rate}}}
\end{align}
$$

## API
The C++ and Python APIs are almost identical, but I will try to highlight the differences here.

**NOTE:** It is recommended to build the `affine_mpc` library in `DEBUG` mode while initially setting up your problem so that you can receive useful error statements regarding usable functions and size of function arguments. Once you have your problem running, then you can
build in `RELEASE` mode to gain some extra speed.

### MPC Constructor
When you create an instance of any MPC class within the library, you must specify the number of states and inputs in your system, the horizon length and number of knot points you want to use in your prediction horizon, and the options you wish to use in your cost and constraint functions (shown [above](#mpc-problem) with red labels). Note that all of the cost and constraint options default to `false`. Once you specify all of these values in the constructor, those values can not change. All of the applicable values in the cost and constraint functions can be changed, but not the size and setup of the MPC problem.

#### Function Declaration

```cpp
ImplicitMPC(const int num_states, const int num_inputs,
            const int horizon_length, const int num_knot_points,
            const bool use_input_cost = false,
            const bool use_slew_rate = false,
            const bool saturate_states = false)
```

#### Usage

**C++**

```cpp
int num_states{2}, num_inputs{1}, horizon{10}, num_knot_points{3};
bool use_input_cost{true}, use_slew_rate{false}, saturate_states{true};
mpc = ImplicitMPC(num_states, num_inputs, horizon, num_knot_points,
                  use_input_cost, use_slew_rate, saturate_states);
```

**Python**

```py
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

```py
calcNextInput( x0[, u] ) -> (u, solved)
calcInputTrajectory( x0[, u_traj] ) -> (u_traj, solved)
```

After solving, you can update any of the parameters before solving again (if you want to change your model, the weights, reference trajectories, etc.). Then you write a loop that will continuously pass in the current state and solve for inputs.
