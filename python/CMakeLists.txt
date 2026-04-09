cmake_minimum_required(VERSION 3.18)
project(affine_mpc_py LANGUAGES CXX)

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
