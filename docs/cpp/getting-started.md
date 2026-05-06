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

### Prerequisites

- C++17 compatible compiler (e.g. [GCC] 8+, [Clang] 5+, [MSVC] 2017+)
- [CMake] 3.15+

!!! note

    Windows users may find the [VS Code guide] helpful for setting up their environment.

[GCC]: https://gcc.gnu.org/
[Clang]: https://clang.llvm.org/
[MSVC]: https://visualstudio.microsoft.com/
[VS Code guide]: https://code.visualstudio.com/docs/cpp/config-msvc
[CMake]: https://cmake.org/

### Required

- [Eigen](https://eigen.tuxfamily.org/dox/GettingStarted.html) 3.4+ (fetched if not on system)
- [OSQP](https://osqp.org/docs/get_started/) 1.0+ (fetched if not on system)

All of these can be installed from source (see their docs), but here are installation instructions for some common operating systems:

=== "Arch"

    ```sh
    sudo pacman -S --asdeps eigen osqp
    ```

=== "Debian/Ubuntu"

    ```sh
    sudo apt install libeigen3-dev
    ```

    ??? note "Note - OSQP Installation"

        OSQP is not currently provided by `apt`.
        You will have to install OSQP from [source](https://github.com/osqp/osqp) to use a system library, or let this project fetch OSQP locally.

    ??? note "Note - Eigen Version"

        Ubuntu 22.04 is the oldest version that satisfies the Eigen version requirement.
        If you are using an older version then you will have to build from [source](https://gitlab.com/), or let this project fetch Eigen locally.

=== "MacOS"

    ```sh
    brew install eigen osqp
    ```

=== "Windows"

    ```pwsh
    C:\vcpkg\vcpkg install eigen3:x64-windows osqp:x64-windows
    ```

    ??? note "Note - vcpkg Installation"

        You can use other tools, but `vcpkg` is shown here since it is the
        official Microsoft package manager.
        To install `vcpkg` to `C:\vcpkg`:

        ```pwsh
        git clone https://github.com/microsoft/vcpkg.git C:\vcpkg
        C:\vcpkg\bootstrap-vcpkg.bat
        ```

### Optional

- [ZLIB](https://zlib.net/) (used for compressed NPZ logging when available)
- [GTest](https://google.github.io/googletest/) for unit tests (fetched if not on system)

=== "Arch"

    ```sh
    sudo pacman -S --asdeps zlib gtest
    ```

=== "Debian/Ubuntu"

    ```sh
    sudo apt install zlib1g-dev libgtest-dev
    ```

=== "MacOS"

    ```sh
    brew install zlib googletest
    ```

=== "Windows"

    ```pwsh
    C:\vcpkg\vcpkg install zlib:x64-windows gtest:x64-windows
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
