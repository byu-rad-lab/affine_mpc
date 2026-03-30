# affine_mpc

`affine_mpc` is a C++17 library for solving discrete-time affine model predictive control (MPC) problems with the OSQP quadratic programming solver. It supports both condensed and sparse formulations and includes B-spline-based input trajectory parameterization, optional constraints, and high-rate binary logging for simulation and debugging workflows.

## Highlights

- Condensed and sparse MPC formulations behind a common interface
- B-spline, linear interpolation, and move-blocking input parameterizations
- Optional input cost, state bounds, input bounds, and slew-rate constraints
- OSQP-backed runtime updates without full solver re-initialization when sparsity is unchanged
- Binary `.npz` logging via `MPCLogger` for Python plotting and analysis

## Quick Start

### Dependencies

Required:

- [Eigen](https://eigen.tuxfamily.org/dox/GettingStarted.html) 3.4+
- [OSQP](https://osqp.org/docs/get_started/) 1.0+
- [cnpy](https://github.com/rogersce/cnpy.git)
- [ZLIB](https://zlib.net/)

Optional:

- [GTest](https://google.github.io/googletest/) for unit tests

If system installs are unavailable, CMake will fetch Eigen, OSQP, cnpy, and GTest as needed.

### Build

```sh
cmake -S . -B build -DAFFINE_MPC_BUILD_TESTS=ON -DAFFINE_MPC_BUILD_EXAMPLES=ON
cmake --build build --config Release --parallel
```

Run the example:

```sh
./build/example_sim
```

Run all tests:

```sh
ctest --test-dir build --output-on-failure
```

Run one test executable:

```sh
ctest --test-dir build -R condensed_mpc_tests --output-on-failure
```

Run one GTest case directly:

```sh
./build/condensed_mpc_tests --gtest_filter=CondensedMPCProtectedTester.givenModel_FormsSandVcorrectly
```

## Minimal Example

```cpp
#include "affine_mpc/affine_mpc.hpp"

int main()
{
  const int n = 2;
  const int m = 1;
  const int T = 10;
  const int nc = 3;
  const double dt = 0.1;

  auto param = affine_mpc::Parameterization::linearInterp(T, nc);
  affine_mpc::CondensedMPC mpc{n, m, param};

  Eigen::Matrix2d A;
  Eigen::Vector2d B, w;
  A << 0, 1, -0.6, -0.1;
  B << 0, 0.2;
  w.setZero();

  mpc.setModelContinuous2Discrete(A, B, w, dt);
  mpc.setInputLimits(Eigen::VectorXd::Constant(m, 0.0),
                     Eigen::VectorXd::Constant(m, 3.0));
  mpc.setStateWeights(Eigen::Vector2d{1.0, 0.1});
  mpc.setReferenceState(Eigen::Vector2d{1.0, 0.0});

  if (!mpc.initializeSolver()) {
    return 1;
  }

  Eigen::Vector2d xk = Eigen::Vector2d::Zero();
  if (mpc.solve(xk) != affine_mpc::SolveStatus::Success) {
    return 1;
  }

  Eigen::VectorXd uk{m};
  mpc.getNextInput(uk);
  return 0;
}
```

## Documentation

- `docs/getting-started.md` - installation, build, tests, and first run
- `docs/concepts.md` - supported MPC formulation, parameterization, solver constraints, and mathematical notation
- `docs/api-overview.md` - recommended usage flow and public API structure
- `docs/examples.md` - example application and plotting workflow
- `docs/logging.md` - `MPCLogger` outputs, metadata, and analysis notes
- `docs/development.md` - repo layout, formatting, testing, and contribution workflow

If you are evaluating the library for research use or publication, `docs/concepts.md` contains the full optimization formulation and definitions of the supported optional costs and constraints.

## Important Usage Notes

- Fully configure the model, limits, weights, and references before calling `initializeSolver()`.
- OSQP sparsity is fixed at initialization. Values may change later, but new nonzero structure must not appear.
- If a matrix entry may become nonzero in a later update, make it nonzero before initialization.
- Prefer `CondensedMPC` unless you specifically need the sparse formulation.

## Examples and Plots

The main C++ example is `examples/sim.cpp`. After running `./build/example_sim`, you can inspect the generated log with:

```sh
python examples/plot_sim_tracking.py
python examples/plot_sim_predictions.py
python examples/plot_npz.py /tmp/ampc_example
```

## Community Standards

- Contributing guide: `CONTRIBUTING.md`
- Code of conduct: `CODE_OF_CONDUCT.md`
- Citation metadata: `CITATION.cff`

## Citation

If you use `affine_mpc` in research, please cite the project using the metadata in `CITATION.cff`.

## License

This project is licensed under the BSD 3-Clause License. See `LICENSE` for details.
