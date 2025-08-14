cmake_minimum_required(VERSION 3.14)
project(affine_mpc)

# custom options
option(BUILD_TESTS "Set to ON if you want to build the unit tests")
option(BUILD_EXAMPLE "Set to ON if you want to build the C++ example")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# print custom options
message(STATUS "BUILD_TESTS: ${BUILD_TESTS}")
message(STATUS "BUILD_EXAMPLE: ${BUILD_EXAMPLE}")

# cmake-gui (ccmake) setup
set(CMAKE_CONFIGURATION_TYPES None Debug Release CACHE STRING
    "Build configurations" FORCE)
if(NOT CMAKE_BUILD_TYPE AND NOT MSVC AND NOT XCODE_VERSION)
  set(CMAKE_BUILD_TYPE None CACHE STRING
      "Possible types are: ${CMAKE_CONFIGURATION_TYPES}")
endif()
set_property(CACHE CMAKE_BUILD_TYPE PROPERTY
  STRINGS ${CMAKE_CONFIGURATION_TYPES})
message(STATUS "CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}")

# dependencies
find_package(Eigen3 3.4 REQUIRED)
message(STATUS "Eigen3 version: ${Eigen3_VERSION}")
include(FetchContent)
include(cmake/extern_osqp.cmake)

# C++ library

# ------------------------------
# static library
# ------------------------------
add_library(${PROJECT_NAME}_static STATIC
  src/mpc_base.cpp
  src/osqp_solver.cpp
  src/implicit_mpc.cpp
  src/bspline_mpc.cpp
  src/mpc_logger.cpp
)
target_include_directories(${PROJECT_NAME}_static PUBLIC include)
target_link_libraries(${PROJECT_NAME}_static PUBLIC
  Eigen3::Eigen
  osqp::osqpstatic
  ${CMAKE_DL_LIBS}
)
# build error without these compile options when building affine_mpc_py
target_compile_options(${PROJECT_NAME}_static PRIVATE -fPIC -std=c++17)

# ------------------------------
# shared library
# ------------------------------
add_library(${PROJECT_NAME} SHARED
  src/mpc_base.cpp
  src/osqp_solver.cpp
  src/implicit_mpc.cpp
  src/bspline_mpc.cpp
  src/mpc_logger.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC include)
target_link_libraries(${PROJECT_NAME} PUBLIC
  Eigen3::Eigen
  osqp::osqp
)
# build error without these compile options when building affine_mpc_py
target_compile_options(${PROJECT_NAME} PRIVATE -fPIC -std=c++17)

# C++ Example
if(BUILD_EXAMPLE)
  add_executable(example_sim examples/sim.cpp)
  target_link_libraries(example_sim PRIVATE ${PROJECT_NAME})
endif()

# C++ library tests
if(BUILD_TESTS)
  enable_testing()
  find_package(GTest REQUIRED)
  message(STATUS "GTest version: ${GTest_VERSION}")
  add_executable(${PROJECT_NAME}_UnitTests
    test/affine_mpc/mpc_base_tests.cpp
    test/affine_mpc/osqp_solver_tests.cpp
    test/affine_mpc/implicit_mpc_tests.cpp
    test/affine_mpc/bspline_mpc_tests.cpp
  )
  target_link_libraries(${PROJECT_NAME}_UnitTests
    gtest
    gtest_main
    pthread
    ${PROJECT_NAME}
  )
  add_test(NAME ${PROJECT_NAME}_UnitTests COMMAND ${PROJECT_NAME}_UnitTests)
  file(GLOB TEST_SOURCES "test/affine_mpc/*.cpp")
  foreach(TEST_SRC ${TEST_SOURCES})
    get_filename_component(TEST_NAME "${TEST_SRC}" NAME_WE)
    add_executable(${TEST_NAME} ${TEST_SRC})
    target_link_libraries(${TEST_NAME}
      gtest
      gtest_main
      pthread
      ${PROJECT_NAME}
    )
    add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
  endforeach()
endif()

