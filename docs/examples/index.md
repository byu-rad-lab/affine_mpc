# Examples

The repository includes the following examples under `examples/`.
Start with the mass-spring-damper; the others each focus on one feature.

| Example | Interface | Demonstrates |
| --- | --- | --- |
| [Mass-Spring-Damper](mass-spring-damper.md) | C++ and Python | The basic workflow: parameterization, configuration, closed-loop solves, logging, and plotting |
| [Quadrotor Model Updates](quadrotor-model-updates.md) | Python | Updating the affine model of a nonlinear plant every step, moving that work before the measurement with `prepare()`, and logging runs for comparison |
| [Laguerre Basis](laguerre.md) | Python | A custom (non-B-spline) basis through `Parameterization.fromBasis()`, in the deviation form |

Install the library first ([C++](../getting-started/cpp.md) or [Python](../getting-started/python.md)); each example page shows how to run it.
Plotting is Python-only, including for logs written by the C++ example (see [Python Getting Started](../getting-started/python.md#plot-the-results)).
