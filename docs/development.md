# Development Guide

This guide summarizes the repository layout, formatting and testing workflow, and the main conventions contributors should follow.

## Repository Layout

- `include/affine_mpc/` - public C++ API headers
- `src/` - library implementation
- `python/affine_mpc/` - Python package sources
- `python/bindings/` - pybind11 binding implementation
- `test/affine_mpc/` - unit tests
- `test/bindings/` - Python binding tests
- `test/python_npz/` - Python/NumPy tests for NPZ fixtures and archive compatibility
- `examples/` - example simulation and plotting scripts
- `docs/` - user-facing documentation and Zensical source
- `cmake/` - dependency and package configuration helpers

## Prerequisites

Some build, test, stub-generation, and docs workflows rely on Python tooling.
It is recommended to use a virtual environment for Python dependencies,
but you can also install them globally if you prefer.

```sh
python -m venv .venv
# Linux/macOS
source .venv/bin/activate
# Windows - use the script for your shell, e.g.:
.venv\Scripts\activate.bat # for cmd.exe
.venv\Scripts\Activate.ps1 # for PowerShell
```

!!! note

    The rest of this guide assumes you have activated a Python virtual environment.
    With a venv activated, `pip` should install packages to the venv rather than globally.
    In some cases, you may need to use `python -m pip` instead of `pip` to ensure the correct Python environment is used.

Install the normal build-time Python dependencies with:

```sh
pip install -r requirements/build.txt
```

Optional additions:

- for docs builds: `pip install -r requirements/docs.txt`
- for Python package and stub development: `pip install ".[dev]"`

## Build Commands

Configure with tests, examples, and bindings (all default to OFF):

```sh
cmake -S . -B build \
    -DCMAKE_BUILD_TYPE=Release \
    -DAFFINE_MPC_BUILD_TESTS=ON \
    -DAFFINE_MPC_BUILD_EXAMPLES=ON \
    -DAFFINE_MPC_BUILD_BINDINGS=ON
```

Build:

```sh
cmake --build build --parallel
```

For a debug build, set `-DCMAKE_BUILD_TYPE=Debug` during configuration.

If you use a multi-config generator such as Visual Studio, select the configuration at build or test time with `--config Release` or `--config Debug`.

## Test Commands

If you configured with `-DAFFINE_MPC_BUILD_TESTS=ON`, run all registered tests with:

```sh
ctest --test-dir build --output-on-failure
```

This covers:

- discovered C++ GoogleTest cases
- `python_npz_tests`
- `python_bindings_tests` if bindings were also enabled

Run the Python test groups directly when you want faster iteration on those suites:

```sh
pytest test/bindings
pytest test/python_npz
```

Notes:

- `test/bindings/` uses `test/bindings/conftest.py` to locate a repo-local package under `install/`, `build/python/`, or `out/Debug/python/`
- `test/bindings/` requires that the bindings have already been built or installed
- `test/python_npz/` uses `test/python_npz/conftest.py` to locate `np_writer_fixture` under `build/` or `out/Debug/`
- `test/python_npz/` does not import the `affine_mpc` Python package, but it does require that tests have been built so the fixture executable exists

Run one discovered CTest by regex:

```sh
ctest --test-dir build -R "MPCLoggerTest\\." --output-on-failure
```

Run one test case:

```sh
./build/mpc_logger_tests --gtest_filter=MPCLoggerTest.ConvenienceLogStepWorks
```

## CMake Configuration

This project defines the following CMake options (specified with `-D`):

| Option                      | Description                                              |
| --------------------------- | -------------------------------------------------------- |
| `CMAKE_BUILD_TYPE`          | Select `Release` or `Debug` for single-config generators |
| `AFFINE_MPC_BUILD_EXAMPLES` | Build example executables                                |
| `AFFINE_MPC_BUILD_TESTS`    | Build unit tests and register test entry points          |
| `AFFINE_MPC_BUILD_BINDINGS` | Build the Python bindings                                |
| `AFFINE_MPC_PY_ADD_TESTS`   | Register Python binding tests in the Python subproject   |

`AFFINE_MPC_PY_ADD_TESTS` is mainly set internally by the top-level build when both bindings and tests are enabled.

## Stub Generation

If you change the Python binding surface, regenerate `python/affine_mpc/_bindings.pyi` and commit the updated file.

From a bindings-enabled build tree:

```sh
cmake --build build --target python_stubgen
```

You can also run the helper script directly:

```sh
python python/custom_stubgen.py
```

The helper imports the built package from the build tree by default.
It also supports `--pkg-dir=install` if you want to generate stubs from an installed repo-local package.

## Formatting

The project uses `.clang-format` with an LLVM-based configuration.

Format edited files with:

```sh
clang-format -i src/mpc_base.cpp
```

Use `// clang-format off` only when it clearly improves readability for matrix literals or similarly structured data.

For prose-heavy Markdown documentation, prefer one sentence per line.
This makes diffs and review comments more localized without affecting rendered output.

## Coding Conventions

- C++17
- 2-space indentation in the existing codebase
- `PascalCase` for classes and structs
- `camelCase` for methods and functions
- `snake_case` for variables and parameters
- `snake_case_` for member variables
- `Eigen::Ref` for Eigen inputs and writable outputs where appropriate
- `Eigen::Map` for zero-copy views into external memory

## Error Handling Philosophy

- invalid dimensions or bad configuration arguments: throw exceptions
- runtime solve results: return `SolveStatus`
- many precondition checks use `assert()`
- avoid introducing unnecessary dynamic allocation in hot paths

## Testing Conventions

- add or update tests for solver logic, indexing, references, constraints, and API behavior
- use `test/affine_mpc/utils.hpp` helpers such as `expectEigenNear(...)`
- it is normal for tests to derive helper classes from production classes to expose protected behavior
- NPZ output compatibility is validated from Python with `numpy.load(...)` and `zipfile`, not from a C++ archive reader

## Documentation Expectations

Build the rendered docs locally with:

```sh
pip install -r requirements/docs.txt
zensical build
```

Or locally serve the docs with:

```sh
zensical serve
```

When changing user-facing behavior:

- update the public header comments if API semantics changed
- update Python docstrings or stubs if the binding surface changed
- update `README.md` if the quick-start path changed
- update the appropriate file under `docs/`

## Contribution Workflow

Before opening a PR:

1. Format edited C++ files with clang-format and Python files with black.
1. For code changes, build with tests, examples, and bindings enabled to make sure the full configuration still works.
1. For docs-only changes, building the code is optional.
1. Run the relevant tests, and run the full test suite when changes affect shared infrastructure, bindings, packaging, or public APIs.
1. Update docs for public behavior changes.
1. Ensure consistency in docs/behavior between C++ and Python (including stubs!).
1. Keep changes focused and easy to review.

See `CONTRIBUTING.md` for the broader contribution process.
