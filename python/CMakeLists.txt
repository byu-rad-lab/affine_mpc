cmake_minimum_required(VERSION 3.18)
project(affine_mpc_py LANGUAGES CXX)

# Stub files provide IDE support (signature help) for compiled bindings
option(AFFINE_MPC_PY_GENERATE_STUBS
  "Generate stub files for compiled bindings" OFF)

# -----------------------------------------
# Dependencies
# -----------------------------------------
# Use the vcpkg toolchain file if defined (for the Github workflow)
if(DEFINED ENV{VCPKG_TOOLCHAIN_FILE})
  set(CMAKE_TOOLCHAIN_FILE $ENV{VCPKG_TOOLCHAIN_FILE} CACHE STRING "")
endif()

set(PYBIND11_FINDPYTHON ON)
find_package(pybind11 2.11.1 CONFIG REQUIRED)

if(NOT TARGET affine_mpc::affine_mpc)
  find_package(affine_mpc CONFIG REQUIRED)
endif()

# -----------------------------------------
# Python bindings
# -----------------------------------------
pybind11_add_module(_bindings MODULE
  bindings/affine_mpc_py_module.cpp
  bindings/py_options.cpp
  bindings/py_parameterization.cpp
  bindings/py_osqp_settings.cpp
  bindings/py_solve_status.cpp
  bindings/py_mpc_base.cpp
  bindings/py_condensed_mpc.cpp
  bindings/py_sparse_mpc.cpp
  bindings/py_mpc_logger.cpp
)

# Set output directory to match package structure
set_target_properties(_bindings PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/affine_mpc"
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/affine_mpc"
)

# Link to the affine_mpc library
target_link_libraries(_bindings PRIVATE affine_mpc::affine_mpc)

# -----------------------------------------
# Stub generation
# -----------------------------------------
if(AFFINE_MPC_PY_GENERATE_STUBS)
  # Locate build environment dependencies
  set(get_stubgen_path
    "import os"
    "import pybind11_stubgen"
    "print(os.path.dirname(os.path.dirname(pybind11_stubgen.__file__)))"
  )
  execute_process(
    COMMAND "${Python_EXECUTABLE}" -c "${get_stubgen_path}"
    OUTPUT_VARIABLE build_env_path
    OUTPUT_STRIP_TRAILING_WHITESPACE
    RESULT_VARIABLE _pystub_found
  )

  # Ensure build environment dependencies were found
  if(NOT _pystub_found EQUAL 0)
    message(WARNING "Could not locate pybind11-stubgen in build environment."
      " Stub generation may fail."
    )
  endif()

  # Allow PYTHONPATH to find both build environment and built module
  if(WIN32)
    set(pypath_separator ";")
  else()
    set(pypath_separator ":")
  endif()
  set(py_paths
    "${build_env_path}${pypath_separator}${CMAKE_CURRENT_BINARY_DIR}"
  )

  # Path to custom stub generation script
  set(gen_stubs "${CMAKE_CURRENT_SOURCE_DIR}/custom_stubgen.py")

  # Collect package Python files explicitly since wildcard expansion in
  # add_custom_command COMMAND arguments is not portable, especially on Windows.
  file(GLOB affine_mpc_py_files "${CMAKE_CURRENT_SOURCE_DIR}/affine_mpc/*.py")

  # Post-build commands to copy package files and generate stubs.
  add_custom_command(TARGET _bindings POST_BUILD
    # Need __init__.py for package recognition and anything it references.
    COMMAND ${CMAKE_COMMAND} -E copy
      ${affine_mpc_py_files}
      ${CMAKE_CURRENT_BINARY_DIR}/affine_mpc/.

    # Generate stubs.
    COMMAND ${CMAKE_COMMAND} -E env "PYTHONPATH=${py_paths}"
      "${Python_EXECUTABLE}" "${gen_stubs}" "${CMAKE_CURRENT_BINARY_DIR}"
  )
endif()

# -----------------------------------------
# Installation
# -----------------------------------------

# Install the compiled bindings module
install(TARGETS _bindings
  DESTINATION affine_mpc
  COMPONENT python_bindings
)

# When using shared libs, python must be able to find the .so files:
#   - if using system libs then they should be in standard findable paths.
#   - else the .so files must be copied into the python environment and the
#     runtime path (RPATH) of _bindings must be set to look in that same
#     location. Windows does not handle RPATH the same way, so using system libs
#     may not work on Windows builds.
if(BUILD_SHARED_LIBS)
  # These libs are aliased when built by this project rather than imported from
  # a system package with find_package. If built by this project they must be
  # installed to the python environment.
  get_target_property(affine_mpc_is_aliased
    affine_mpc::affine_mpc ALIASED_TARGET
  )
  get_target_property(osqp_is_aliased osqp::osqp ALIASED_TARGET)

  if(affine_mpc_is_aliased OR osqp_is_aliased)
    # This means a .so file will be copied into the python environment
    # and the RPATH must know to look for it there.
    if(APPLE)
      set_target_properties(_bindings PROPERTIES INSTALL_RPATH "@loader_path")
    elseif(UNIX)
      set_target_properties(_bindings PROPERTIES INSTALL_RPATH "$ORIGIN")
    endif()
  endif()

  if(affine_mpc_is_aliased)
    install(TARGETS affine_mpc
      RUNTIME DESTINATION affine_mpc
      LIBRARY DESTINATION affine_mpc
      COMPONENT python_bindings
    )
  else()
    message(STATUS "Using affine_mpc shared lib found from system.")
  endif()

  if(osqp_is_aliased)
    install(TARGETS osqp
      RUNTIME DESTINATION affine_mpc
      LIBRARY DESTINATION affine_mpc
      COMPONENT python_bindings
    )
  else()
    message(STATUS "Using OSQP shared lib found from system.")
  endif()
endif()

## Use this if stub file should be installed by build
## (if _bindings.pyi is removed from the repo)
# if(AFFINE_MPC_PY_GENERATE_STUBS)
#   # Install generated stub file for the bindings module (IDE support)
#   install(FILES ${CMAKE_CURRENT_BINARY_DIR}/stubs/affine_mpc/_bindings.pyi
#     DESTINATION affine_mpc
#     COMPONENT python_bindings
#     OPTIONAL
#   )
# endif()

# Install package Python and stub files
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/affine_mpc
  DESTINATION .
  COMPONENT python_bindings
  FILES_MATCHING
    PATTERN "*.py"
    PATTERN "*.pyi"
    PATTERN "py.typed"
)

# Install example files
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../examples/
  DESTINATION affine_mpc/_examples
  COMPONENT python_bindings
  FILES_MATCHING
    PATTERN "*.py"
)
