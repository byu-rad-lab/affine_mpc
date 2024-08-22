cmake_minimum_required(VERSION 3.14)
project(affine_mpc)

# custom options
option(BUILD_TESTS "Set to ON if you want to build the unit tests")
option(BUILD_PYTHON "Set to ON if you want to build the Python bindings")
set(PYBIND11_VER "2.9.2" CACHE STRING "Set desired version of Pybind11")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# print custom options
message(STATUS "BUILD_TESTS: ${BUILD_TESTS}")
message(STATUS "BUILD_PYTHON: ${BUILD_PYTHON}")

# cmake-gui (ccmake) setup
set(CMAKE_CONFIGURATION_TYPES None Debug Release CACHE STRING "Build configurations" FORCE)
if(NOT CMAKE_BUILD_TYPE AND NOT MSVC AND NOT XCODE_VERSION)
  set(CMAKE_BUILD_TYPE None CACHE STRING
      "Possible types are: ${CMAKE_CONFIGURATION_TYPES}")
endif()
set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS ${CMAKE_CONFIGURATION_TYPES})

# dependencies
find_package(Eigen3 3.3.7 REQUIRED)
message(STATUS "Eigen3 version: ${Eigen3_VERSION}")
include(FetchContent)
include(cmake/extern_osqp.cmake)


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

if (BUILD_TESTS)
  enable_testing()
  find_package(GTest REQUIRED)
  add_executable(${PROJECT_NAME}_UnitTests
    test/mpc_base_tests.cpp
    test/osqp_solver_tests.cpp
    test/implicit_mpc_tests.cpp
  )
  target_link_libraries(${PROJECT_NAME}_UnitTests
    gtest
    gtest_main
    pthread
    ${PROJECT_NAME}
  )
  add_test(NAME ${PROJECT_NAME}_UnitTests COMMAND ${PROJECT_NAME}_UnitTests)
endif()


if (BUILD_PYTHON)
  include(cmake/extern_pybind11.cmake)
  pybind11_add_module(${PROJECT_NAME}_py
    python/affine_mpc_py_module.cpp
    python/py_mpc_base.cpp
    python/py_implicit_mpc.cpp
    python/py_mpc_logger.cpp
    python/py_osqp_settings.cpp
  )
  target_link_libraries(${PROJECT_NAME}_py PUBLIC ${PROJECT_NAME})
endif()
