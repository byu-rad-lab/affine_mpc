# Contributing to affine_mpc

First off, thank you for considering contributing to `affine_mpc`!
It’s people like you that make open-source research software better.

## How Can I Contribute?

### Reporting Bugs

- Check the existing issues to see if the bug has already been reported.
- If not, open a new issue. Include a clear title, a description of the problem, and a minimal working example (C++ or Python) that reproduces the issue.

### Suggesting Enhancements

- Open an issue to discuss your proposed feature before starting work.
- Clearly describe the research use case for the enhancement.

### Pull Requests

1.  Fork the repo and create your branch from `main`.
2.  Ensure your code follows the existing style (we use `.clang-format`).
3.  **Tests are required.** Any new feature or bug fix must include corresponding unit tests in the `test/` directory.
4.  Update the documentation (Doxygen and README) as needed.
5.  Issue your pull request!

## Development Environment

- Target C++ standard: **C++17**.
- Build system: **CMake**.
- Dependency management: We try to use system packages and fallback to `FetchContent` for external dependencies (Eigen, OSQP, cnpy).

## Review Process

All pull requests will be reviewed by the maintainers. We look for:

- Mathematical correctness.
- Modern C++ best practices (RAII, memory safety).
- Test coverage and pass rates.
- Clear documentation.

---

By contributing, you agree that your contributions will be licensed under the project's [BSD 3-Clause License](LICENSE).
