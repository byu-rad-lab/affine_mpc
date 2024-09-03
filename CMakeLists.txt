cmake_minimum_required(VERSION 3.14)
project(affine_mpc)

# custom options
option(BUILD_TESTS "Set to ON if you want to build the unit tests")
option(BUILD_BINDINGS "Set to ON if you want to build the Python bindings")
option(BUILD_EXAMPLE "Set to ON if you want to build the C++ example")
set(PYBIND11_VERSION "2.9.2" CACHE STRING "Set desired version of Pybind11")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# print custom options
message(STATUS "BUILD_TESTS: ${BUILD_TESTS}")
message(STATUS "BUILD_BINDINGS: ${BUILD_BINDINGS}")
message(STATUS "BUILD_EXAMPLE: ${BUILD_EXAMPLE}")

# cmake-gui (ccmake) setup
set(CMAKE_CONFIGURATION_TYPES None Debug Release CACHE STRING "Build configurations" FORCE)
if (NOT CMAKE_BUILD_TYPE AND NOT MSVC AND NOT XCODE_VERSION)
  set(CMAKE_BUILD_TYPE None CACHE STRING
      "Possible types are: ${CMAKE_CONFIGURATION_TYPES}")
endif ()
set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS ${CMAKE_CONFIGURATION_TYPES})
message(STATUS "CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}")

# dependencies
find_package(Eigen3 3.3.7 REQUIRED)
message(STATUS "Eigen3 version: ${Eigen3_VERSION}")
include(FetchContent)
include(cmake/extern_osqp.cmake)

# C++ library
add_library(${PROJECT_NAME} STATIC
  src/mpc_base.cpp
  src/osqp_solver.cpp
  src/implicit_mpc.cpp
  src/mpc_logger.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC include)
target_link_libraries(${PROJECT_NAME} PUBLIC
  Eigen3::Eigen
  osqp::osqp
)
target_compile_options(${PROJECT_NAME} PRIVATE -fPIC -std=c++17) # build error without this when building affine_mpc_py

# C++ Example
if (BUILD_EXAMPLE)
  add_executable(example_sim examples/sim.cpp)
  target_link_libraries(example_sim PRIVATE ${PROJECT_NAME})
endif (BUILD_EXAMPLE)

# C++ library tests
if (BUILD_TESTS)
  enable_testing()
  find_package(GTest REQUIRED)
  message(STATUS "GTest version: ${GTest_VERSION}")
  add_executable(${PROJECT_NAME}_UnitTests
    test/affine_mpc/mpc_base_tests.cpp
    test/affine_mpc/osqp_solver_tests.cpp
    test/affine_mpc/implicit_mpc_tests.cpp
  )
  target_link_libraries(${PROJECT_NAME}_UnitTests
    gtest
    gtest_main
    pthread
    ${PROJECT_NAME}
  )
  add_test(NAME ${PROJECT_NAME}_UnitTests COMMAND ${PROJECT_NAME}_UnitTests)
endif (BUILD_TESTS)

# Python bindings for C++ library
if (BUILD_BINDINGS)
  include(cmake/extern_pybind11.cmake)
  pybind11_add_module(_bindings MODULE
    python/bindings/affine_mpc_py_module.cpp
    python/bindings/py_mpc_base.cpp
    python/bindings/py_implicit_mpc.cpp
    python/bindings/py_mpc_logger.cpp
    python/bindings/py_osqp_settings.cpp
  )
  target_link_libraries(_bindings PUBLIC ${PROJECT_NAME})
  set_target_properties(_bindings PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/python/affine_mpc_py
  )

  # Only build python tests if we are building the python bindings
  if (BUILD_TESTS)
    # Add target for use with `cmake --build . --target test_bindings`
    add_custom_target(test_bindings
      COMMAND pytest
      WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    )
    # Add test for use with `ctest`
    add_test(NAME test_bindings COMMAND pytest WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
  endif (BUILD_TESTS)
endif (BUILD_BINDINGS)
