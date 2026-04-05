# C++ Getting Started

This guide gets you to a first successful build and run with the C++ library.

## Requirements

The project targets:

- C++17
- CMake 3.15+

Required dependencies:

- [Eigen](https://eigen.tuxfamily.org/dox/GettingStarted.html) 3.4+
- [OSQP](https://osqp.org/docs/get_started/) 1.0+
- [cnpy](https://github.com/rogersce/cnpy.git)
- [ZLIB](https://zlib.net/)

Optional dependency:

- [GTest](https://google.github.io/googletest/) for unit tests

If system installs are unavailable, CMake will fetch Eigen, OSQP, cnpy, and GTest as needed.

## Build

Configure and build with examples enabled:

```sh
cmake -S . -B build -DAFFINE_MPC_BUILD_TESTS=OFF -DAFFINE_MPC_BUILD_EXAMPLES=ON
cmake --build build --config Release --parallel
```

Build with tests enabled:

```sh
cmake -S . -B build -DAFFINE_MPC_BUILD_TESTS=ON -DAFFINE_MPC_BUILD_EXAMPLES=ON
cmake --build build --config Release --parallel
```

Debug builds are useful because Eigen performs additional runtime checks:

```sh
cmake -S . -B out/Debug -G Ninja -DCMAKE_BUILD_TYPE=Debug -DAFFINE_MPC_BUILD_TESTS=ON -DAFFINE_MPC_BUILD_EXAMPLES=ON
cmake --build out/Debug --parallel
```

## Run Tests

Run all discovered tests:

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

## Run the Example Application

Build with examples enabled, then run:

```sh
./build/example_sim
```

The example simulates a mass-spring-damper system and writes an `.npz` log and YAML parameter file into the system temporary directory under `ampc_example`.

Expected outcome:

- the executable runs a closed-loop tracking simulation
- `log.npz` and `params.yaml` are written under the `ampc_example` temp directory
- the log can be inspected with the Python plotting helper

To inspect the log from Python:

```sh
python examples/plot_sim.py
```

For the intended API workflow, see [Usage](usage.md).
