# C++ Getting Started

This guide gets you to a first successful build and run with the C++ library.

## Quickstart

The fastest path is usually to try building first and let CMake fetch missing dependencies if needed:

```sh
cmake -S . -B build -DAFFINE_MPC_BUILD_EXAMPLES=ON
cmake --build build --config Release --parallel
./build/example_sim
```

This should build and run the `example_sim` executable, see [Expected Results](#expected-results).

If you prefer to use system libraries, the expected dependencies are listed below.

## Dependencies

The project targets:

- C++17
- CMake 3.15+

Required dependencies:

- [Eigen](https://eigen.tuxfamily.org/dox/GettingStarted.html) 3.4+ (fetched if not on system)
- [OSQP](https://osqp.org/docs/get_started/) 1.0+ (fetched if not on system)
- [cnpy](https://github.com/rogersce/cnpy.git) (always fetched)
- [ZLIB](https://zlib.net/) (must be on system - commonly is for Linux/MacOS)

All of these can be installed from source (see their docs), but here are installation instructions for some common operating systems:

=== "Arch"

    ```sh
    sudo pacman -S --asdeps eigen osqp zlib
    ```

=== "Ubuntu/Debian"

    ```sh
    sudo apt install libeigen3-dev zlib1g-dev
    ```

    !!! note

        OSQP is not currently provided by `apt`.
        You will have to install OSQP from [source](https://github.com/osqp/osqp) to use a system library, or let this project fetch OSQP locally.

    !!! note

        Ubuntu 22.04 is the oldest version that satisfies the Eigen version requirement.
        If you are using an older version then you will have to build from [source](https://gitlab.com/), or let this project fetch Eigen locally.

=== "MacOS"

    ```sh
    brew install eigen osqp zlib
    ```

=== "Windows"

    There are likely many ways to install dependencies; using `vcpkg` is shown here:

    If `vcpkg` is not installed (you can install it where you want - this will use `C:\vcpkg`)

    ```pwsh
    git clone https://github.com/microsoft/vcpkg.git C:\vcpkg
    C:\vcpkg\bootstrap-vcpkg.bat
    ```

    Install dependencies (use path to your `vcpkg` installation):

    ```pwsh
    C:\vcpkg\vcpkg install eigen3:x64-windows osqp:x64-windows zlib:x64-windows
    ```

Optional dependency:

- [GTest](https://google.github.io/googletest/) for unit tests (fetched if not on system)

=== "Arch"

    ```sh
    sudo pacman -S --asdeps gtest
    ```

=== "Ubuntu/Debian"

    ```sh
    sudo apt install libgtest-dev
    ```

=== "MacOS"

    ```sh
    brew install googletest
    ```

=== "Windows"

    ```pwsh
    C:\vcpkg\vcpkg install gtest:x64-windows
    ```

## Expected Results

After running the `example_sim` executable from above, you should see:

- The example runs successfully, printing the log path to the console.
- `log.npz` and `params.yaml` are written under the `ampc_example` temp directory.

For more detail on what the example demonstrates and how to interpret the outputs, see [Examples](../examples.md).

!!! note

    The library does not provide visualization capabilities in C++.
    Python is intended to be used for plotting.

To visualize the log from Python, see the plotting workflow in [Python Getting Started](../python/getting-started.md#plot-the-results).

## How to Use in your Project

!!! info

    While developing, you may want to compile in `Debug` mode (`cmake --build build --config Debug --parallel`).
    This library does some additional size checks on Eigen variables in `Debug` mode.
    Once developed though, `Release` mode is _much_ faster.

### Install to System

This is useful if you want to reuse `affine_mpc` in multiple projects.

You can install `affine_mpc` to your system after it has been built (likely need admin privileges):

```sh
cmake --install build
```

Then in your `CMakeLists.txt`:

```cmake
find_package(affine_mpc CONFIG REQUIRED)
```

### Use FetchContent

This option is useful if you only intend on having a single project that uses `affine_mpc`.

Here is an example of what to include in your CMake config:

```cmake
include(FetchContent)
FetchContent_Declare(ampc_extern
  GIT_REPOSITORY https://github.com/byu-rad-lab/affine_mpc
  GIT_TAG        v1.0.0  # (check for latest release)
)

# set options as desired
set(AFFINE_MPC_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(AFFINE_MPC_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(AFFINE_MPC_BUILD_BINDINGS OFF CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(ampc_extern)

# ...

target_link_libraries(<your_target> PRIVATE affine_mpc::affine_mpc)
```

## Next Steps

For the intended API workflow, see [Usage](usage.md).
