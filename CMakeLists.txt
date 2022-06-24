cmake_minimum_required(VERSION 3.0.2)
project(affine_mpc)

option(BUILD_TESTS "Set to ON if you want to build the unit tests")
option(BUILD_PYTHON "Set to ON if you want to build the Python bindings")

message(STATUS "BUILD_TESTS: ${BUILD_TESTS}")
message(STATUS "BUILD_PYTHON: ${BUILD_PYTHON}")

find_package(osqp REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(${PROJECT_NAME} STATIC
  src/mpc_base.cpp
  src/osqp_solver.cpp
  src/implicit_mpc.cpp
  src/mpc_logger.cpp
  )
target_include_directories(${PROJECT_NAME} PUBLIC include)
target_link_libraries(${PROJECT_NAME} PUBLIC
  Eigen3::Eigen
  osqp::osqpstatic
  ${CMAKE_DL_LIBS} # for osqp to work
  )
target_compile_options(${PROJECT_NAME} PRIVATE -fPIC -std=c++17) # build error without this when building affine_mpc_py

if (BUILD_TESTS)
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
endif()

if (BUILD_PYTHON)
  set(PYBIND11_PYTHON_VERSION 3 CACHE STRING "")
  find_package(pybind11 REQUIRED)
  pybind11_add_module(${PROJECT_NAME}_py
    python/affine_mpc_py_module.cpp
    python/py_mpc_base.cpp
    python/py_implicit_mpc.cpp
    python/py_mpc_logger.cpp
    python/py_osqp_settings.cpp
    )
  target_link_libraries(${PROJECT_NAME}_py PUBLIC ${PROJECT_NAME})
endif()
