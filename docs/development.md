# Development Guide

This guide summarizes the repository layout, formatting and testing workflow, and the main conventions contributors should follow.

## Repository Layout

- `include/affine_mpc/` - public C++ API headers
- `src/` - library implementation
- `python/affine_mpc/` - Python package sources
- `python/bindings/` - pybind11 binding implementation
- `test/affine_mpc/` - unit tests
- `test/bindings/` - Python binding tests
- `examples/` - example simulation and plotting scripts
- `docs/` - user-facing documentation and MkDocs source
- `cmake/` - dependency and package configuration helpers

## Build Commands

Configure with tests and examples (both default to OFF):

```sh
cmake -S . -B build -DAFFINE_MPC_BUILD_TESTS=ON -DAFFINE_MPC_BUILD_EXAMPLES=ON
```

Build (specify Release vs Debug mode - Release is much faster):

```sh
cmake --build build --config Release --parallel
# cmake --build build --config Debug --parallel
```

## Test Commands

Run all tests:

```sh
ctest --test-dir build --output-on-failure
```

Run a single executable:

```sh
ctest --test-dir build -R mpc_logger_tests --output-on-failure
```

Run one test case:

```sh
./build/mpc_logger_tests --gtest_filter=MPCLoggerTest.ConvenienceLogStepWorks
```

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

- C++17 only
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

## Documentation Expectations

When changing user-facing behavior:

- update the public header comments if API semantics changed
- update Python docstrings or stubs if the binding surface changed
- update `README.md` if the quick-start path changed
- update the appropriate file under `docs/`

## Contribution Workflow

Before opening a PR:

1. format edited C++ files
2. run the relevant tests
3. update docs for public behavior changes
4. keep changes focused and easy to review

See `CONTRIBUTING.md` for the broader contribution process.
