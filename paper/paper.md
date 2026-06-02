---
title: "affine_mpc: A library for affine model predictive control with C++ and Python interfaces"
tags:
  - Model predictive control
  - Control systems
  - Optimization
  - C++
  - Python
authors:
  - name: Mathew Haskell
    orcid: 0000-0003-3813-7312
    affiliation: 1
  - name: Marc Killpack
    orcid: 0000-0001-9372-104X
    affiliation: 1
affiliations:
  - name: Brigham Young University, 350 EB, Provo, UT 84602, USA
    index: 1
date: 19 March 2026 # TODO: update date (use when the file was created?)
bibliography: paper.bib
---

# Summary

`affine_mpc` is an object-oriented library for model predictive control (MPC) using discrete-time affine models, with C++ and Python interfaces.
It is designed to support real-time control and rapid research prototyping with a focused set of common cost and constraint functions, efficient live parameter updates in a control loop, and binary logging support.

The core of `affine_mpc` is essentially a convenience interface to the OSQP solver [@stellato_osqp_2020] designed specifically for MPC, where it manages the conversion from an MPC optimization to a QP optimization.
For control loops where the reference trajectory, model, or other parameters change between each solve, `affine_mpc` has been designed to try and minimize the computation and memory copies required to achieve these parameter updates for faster solve rates.
All optimization options that can be toggled in the library are available for both sparse and condensed formulations of the QP problem, providing a common interface.

A key feature of `affine_mpc` is its native support for input trajectory parameterization using B-splines, which significantly reduces the number of decision variables in the underlying quadratic program (QP) while allowing for smooth control signals.
This parameterization makes the number of decision variables independent of horizon length and is instead a function of the number of control points in the spline.
This can significantly reduce solve times, especially for longer prediction horizons.

Binary logging is another convenience feature of `affine_mpc`.
Data can be logged to a single NPZ file (compressed or uncompressed), a collection of NPY files, or a collection of raw binary files with header information.
NPZ and NPY files can be easily loaded into Python's Numpy package for data visualization.

# Statement of Need

<!-- TODO: "limit" to robots and UAVs, or say something more general like "For researchers and engineers," -->

In robotics and aerospace engineering, MPC is a standard tool for handling constrained multi-variable optimal control problems.
However, many existing MPC implementations either rely on heavy, general-purpose optimization suites or require significant boilerplate code to map the control problem to a solver-compatible format.
`affine_mpc` addresses this by providing an MPC tool that sits between low-level QP assembly and large general-purpose control frameworks.

`affine_mpc` aims to lower the barrier to entry for developing MPC controllers by reducing the amount of low-level problem assembly needed for common affine, or linear, MPC workflows.
`affine_mpc` focuses on discrete-time affine MPC problems with optional costs and constraints, efficient repeated solves, and workflows that support both experimentation in Python and integration in C++.

By integrating B-spline parameterization natively into the QP formulation, researchers can easily trade off computational complexity against control signal smoothness.
Many common input trajectory parameterization methods [@rossiter_review_2023] are variations of degree 0 B-splines, meaning that the B-spline parameterization within `affine_mpc` provides a unified framework of parameterization that facilitates comparison with traditional parameterization techniques.

# State of the Field

Perhaps the most relevant available software packages are `Acados` [@verschueren_acadosmodular_2022] and `CasADi` [@andersson_casadi_2012], both of which are general-purpose libraries for nonlinear optimal control problems.
`affine_mpc` is a more focused library supporting only affine time-invariant models, where the optimization is convex and the structure is fixed at initialization.
This focused structure allows for a reduction in user boilerplate and yields highly tailored application code rather than having to support generic use cases.
This focused structure significantly reduces user boilerplate while allowing the library itself to be highly optimized for affine models, avoiding the architectural overhead required to support generic nonlinear frameworks.

There is also a lightweight package, somewhat similar to `affine_mpc`, called `osqp-mpc`
[@boylan_jtylerboylanosqp-mpc_2026].
It consists of a single header file that can be used to solve linear MPC problems.
However, it does not directly support affine models, parameterized input trajectories, slew-rate constraints, nor built-in logging capabilities.

[@noauthor_gbionicsosqp-eigen_2026] and [@noauthor_googleosqp-cpp_2026] are both C++ wrappers for `OSQP`, which is written in pure C.
These alone, just like `OSQP` [@stellato_osqp_2020] itself, are general-purpose QP libraries that are not catered directly for MPC.
One of these could have been used in `affine_mpc` as the C++ wrapper, but we did not need all of their functionality and wanted to limit the number of external dependencies.
For these reasons, we implement our own minimal C++ wrapper for `OSQP`.

`libnpy` [@lohse_llohselibnpy_2026] is a C++ library for reading and writing NPY files; however, it does not support NPZ files.
`cnpy` [@rogers_rogerscecnpy_2026] is another C++ library which can read and write both NPY and NPZ files, but it does not support writing compressed NPZ files.
Neither of these libraries support streaming data directly from a binary file to a NPY/NPZ file, which were features we desired for the logging capabilities of `affine_mpc`.

In a MPC parameterization review paper by Rossiter et al. [@rossiter_review_2023], they review various methods for parameterizing the input trajectory in MPC, including two piecewise-constant methods, Laguerre polynomials, and dual MPC.
B-spline parameterization is not included in their review and does not seem to be widely used with discrete-time MPC.
However, B-splines can directly implement both of the piecewise-constant methods as special cases, and it can provide smoothness with higher degrees similar to Laguerre polynomials.
While Laguerre polynomials are global basis functions, B-splines are local basis functions, which can provide finer control over the shape of the trajectory.
We are unaware of any existing software packages that natively support B-spline parameterization for discrete-time MPC, making this a unique feature of `affine_mpc`.

# Software Design

Careful consideration was used to try and minimize the amount of memory copies, computation, and conditional checks that occur when updating any part of the MPC problem (model, weights, references, limits, etc.).
We found that sparse and condensed formulations differ enough in how they are effected by those updates, that we separated them out into distinct classes `SparseMPC` and `CondensedMPC`.
Shared functionality is defined in the abstract `MPCBase` class.
For a consistent API, `MPCBase` implements all public interface methods for setting the various MPC parameters, while both derived classes implement the various private methods that update the QP problem.

We chose to use a configure then initialize approach in creating MPC objects.
This means that MPC objects are not fully useable directly after construction.
Users must construct the object, call all relevant public API methods for setting updateable parameters, then call `initializeSolver` before the object is ready to call `solve`.
We chose this approach for the following reasons:

- To provide a consistent interface for setting all the parameters that can be updated between solves
- This helps flush out improper use through exceptions before being in the hot path where `solve` is called repeatedly and exceptions are not desired
- OSQP uses a similar style, making it feel more consistent
- To prevent having a large number of arguments in the MPC constructor

The items that must be provided at construction are those that fix the size and configuration of the MPC problem.
They include the state and input vector dimensions, a `Parameterization`, and `Options`.

For now, all the supported cost and constraint options were simple enough to be implemented with basic `bool` types in the `Options` class.
If more features are added, we could see some of the options changing to enum types to support various modes that are still labeled for clarity.

The `Parameterization` class was designed to try and be intuitive even for those who are unfamiliar with B-splines.
It contains factory methods for move-blocking, linear interpolation, and clamped B-splines where the user mainly needs to specify the horizon length and number of control points used to parameterize the input trajectory.
Nonuniform knots can also be used for finer control of the parameterization.

The `MPCLogger` class was designed to be independent from MPC objects, since it is not necessary.
Thus, we made it a friend class to `MPCBase` to have easy access to log internal data rather than have it be a member pointer that is managed inside of `MPCBase`.
At one point we used `cnpy` [@cnpy] to save Eigen matrices to NPY/NPZ files, but we moved away from this to implementing all logging features internally.
The reasons for this were:

- `cnpy`, and other libraries like it, have extra features we did not need and lacked features we desired
  - It did not support writing compressed NPZ files
  - It did not support streaming directly from a binary file to a NPY/NPZ file (the full matrix had to be loaded in memory whereas we wanted to be able to stream data in chunks)
  - Their CMake configuration and packaging was outdated and the project had not been touched in years even though many pull-requests exist
  - They had required dependencies we did not want to require
  - They installed executables we did not need
  - We did not want to fork, update, and maintain their library with features we do not use
- To minimize required, 3rd-party dependencies, especially if they are not mainstream or appear to not be maintained

# Research Impact

`affine_mpc` was used in a study of the effects of local model approximations on MPC performance [TODO: @haskell_effects_2026].
The approximation point used in affinizing a nonlinear model was found to have a significant effect on the performance of the MPC controller.
Each approximation point led to a different affine model, where `affine_mpc` facilitated the performance comparison between the different models with a simple and consistent interface.

<!-- TODO: finish this section -->

The B-spline parameterization used in `affine_mpc` also enabled a research paper [TODO: @haskell_bsplines_2026] that ...
compared the performance of B-spline parameterization to traditional piecewise-constant parameterization for
discrete-time MPC.

# AI Usage Disclosure

No generative AI was used in the development of the core library, nor for the unit tests related to mathematical correctness.

GPT 5.4 was used to generate boilerplate docstrings, Github workflows, unit tests on exception logic, website documentation, and some of the NPZ/NPY writing features.
All content was reviewed by the authors and generally significantly altered (especially with documentation) to ensure the authors' intent was represented.
