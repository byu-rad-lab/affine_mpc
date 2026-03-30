# Getting Started

This guide walks through the first successful use of `affine_mpc`: installing dependencies, building the library, running tests, and launching the example application.

## Requirements

The project targets:

- C++17
- CMake 3.15+
- GCC on Linux has been the primary development environment

Required dependencies:

- [Eigen](https://eigen.tuxfamily.org/dox/GettingStarted.html) 3.4+ - matrix library
- [OSQP](https://osqp.org/docs/get_started/) 1.0+ - QP library
- [cnpy](https://github.com/rogersce/cnpy.git) - save NPZ log files
- [ZLIB](https://zlib.net/) - compression library to save NPZ log files

Optional dependency:

- [GTest](https://google.github.io/googletest/) - for unit tests

`affine_mpc` prefers system installs when available, but CMake will fetch Eigen, OSQP, cnpy, and GTest automatically if needed.

### Linux package examples

Ubuntu:

```sh
sudo apt install libeigen3-dev libgtest-dev zlib1g-dev
```

Arch:

```sh
sudo pacman -S --asdeps eigen osqp gtest zlib
```

Install from source (option for any other OS - or check your OS's package manager):

- [Eigen](https://gitlab.com/libeigen/eigen)
- [OSQP](https://github.com/osqp/osqp)
- `cnpy` is always fetched by CMake (no system install used)
- `zlib` must be a system install (likely already on your system)
- [GTest](https://github.com/google/googletest)

## Configure and Build

The standard build is out-of-source.

### Build the library and examples

```sh
cmake -S . -B build -DAFFINE_MPC_BUILD_TESTS=OFF -DAFFINE_MPC_BUILD_EXAMPLES=ON
cmake --build build --config Release --parallel
```

### Build with tests enabled

```sh
cmake -S . -B build -DAFFINE_MPC_BUILD_TESTS=ON -DAFFINE_MPC_BUILD_EXAMPLES=ON
cmake --build build --config Release --parallel
```

### Debug build

Debug builds are helpful during development because there are some additional runtime size checks on Eigen arguments.

```sh
cmake -S . -B out/Debug -G Ninja -DCMAKE_BUILD_TYPE=Debug -DAFFINE_MPC_BUILD_TESTS=ON -DAFFINE_MPC_BUILD_EXAMPLES=ON
cmake --build out/Debug --parallel
```

## Run Tests

Run all discovered tests with CTest:

```sh
ctest --test-dir build --output-on-failure
```

Run one test executable:

```sh
ctest --test-dir build -R condensed_mpc_tests --output-on-failure
```

Run one test case directly with GTest:

```sh
./build/condensed_mpc_tests --gtest_filter=CondensedMPCProtectedTester.givenModel_FormsSandVcorrectly
```

## Run the Example Application

Build with examples enabled, then run:

```sh
./build/example_sim
```

The example simulates a mass-spring-damper system and writes an `.npz` log and YAML parameter file into the system temporary directory, typically `/tmp/ampc_example` on Linux.

## Plot the Example Results

After the example runs, use the plotting script:

```sh
python examples/plot_sim.py
```

## First MPC Workflow

The typical order of operations is:

1. Build a `Parameterization`
2. Construct `CondensedMPC` or `SparseMPC`
3. Set the model
4. Set limits required by enabled options
5. Set weights and references
6. Call `initializeSolver()`
7. Call `solve(x0)` in the control loop
8. Retrieve inputs or trajectories

For a fuller walkthrough, see `docs/api-overview.md`.

## Common Gotchas

- Do not call `solve()` before `initializeSolver()`.
- If an option enables a constraint, the corresponding setter must be called before initialization.
- OSQP cannot accept new sparsity after initialization. If a matrix entry may become nonzero later, it must already be nonzero at initialization time.
- Prefer `Release` for performance measurement and `Debug` for diagnosing dimension or shape issues.
