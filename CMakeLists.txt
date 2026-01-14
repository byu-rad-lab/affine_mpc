cmake_minimum_required(VERSION 3.14)
project(affine_mpc LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# custom options
option(BUILD_TESTS "Set to ON if you want to build the unit tests")
option(BUILD_EXAMPLE "Set to ON if you want to build the C++ example")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# need this to update minimum cmake versions for OSQP
set(CMAKE_POLICY_VERSION_MINIMUM 3.10)

# print custom options
message(STATUS "BUILD_TESTS: ${BUILD_TESTS}")
message(STATUS "BUILD_EXAMPLE: ${BUILD_EXAMPLE}")

# cmake-gui (ccmake) setup
set(CMAKE_CONFIGURATION_TYPES None Debug Release CACHE STRING
    "Build configurations" FORCE)
if(NOT CMAKE_BUILD_TYPE AND NOT MSVC AND NOT XCODE_VERSION)
  set(CMAKE_BUILD_TYPE None CACHE STRING
      "Possible types are: ${CMAKE_CONFIGURATION_TYPES}")
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY
    STRINGS ${CMAKE_CONFIGURATION_TYPES})
endif()
message(STATUS "CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}")

# dependencies
include(FetchContent)
include(cmake/extern_eigen.cmake)
include(cmake/extern_osqp.cmake)

# C++ library
set(AMPC_SOURCES
  src/mpc_base.cpp
  src/osqp_solver.cpp
  src/implicit_mpc.cpp
  src/bspline_mpc.cpp
  src/mpc_logger.cpp
)
set(AMPC_DEPS
  Eigen3::Eigen
)

# ------------------------------
# static library
# ------------------------------
add_library(${PROJECT_NAME}_static STATIC ${AMPC_SOURCES})
target_include_directories(${PROJECT_NAME}_static PUBLIC include)
target_link_libraries(${PROJECT_NAME}_static PUBLIC
  ${AMPC_DEPS}
  osqp::osqpstatic
)

# ------------------------------
# shared library
# ------------------------------
add_library(${PROJECT_NAME} SHARED ${AMPC_SOURCES})
target_include_directories(${PROJECT_NAME} PUBLIC include)
target_link_libraries(${PROJECT_NAME} PUBLIC
  ${AMPC_DEPS}
  osqp::osqp
)

# build error without these compile options when building affine_mpc_py
if(NOT MSVC)
  target_compile_options(${PROJECT_NAME}_static PRIVATE -fPIC -std=c++17)
  target_compile_options(${PROJECT_NAME} PRIVATE -fPIC -std=c++17)
endif()

# C++ Example
if(BUILD_EXAMPLE)
  set(EX_SOURCES
    examples/sim.cpp
  )

  add_executable(example_sim_static ${EX_SOURCES})
  target_link_libraries(example_sim_static PRIVATE ${PROJECT_NAME}_static)

  add_executable(example_sim ${EX_SOURCES})
  target_link_libraries(example_sim PRIVATE ${PROJECT_NAME})
endif()

# C++ library tests
if(BUILD_TESTS)
  enable_testing()

  file(GLOB TEST_SOURCES "test/affine_mpc/*.cpp")
  set(TEST_DEPS
    gtest
    gtest_main
    pthread
  )

  include(cmake/extern_gtest.cmake)
  add_executable(${PROJECT_NAME}_UnitTests ${TEST_SOURCES})
  target_link_libraries(${PROJECT_NAME}_UnitTests
    ${TEST_DEPS}
    ${PROJECT_NAME}
  )
  add_test(NAME ${PROJECT_NAME}_UnitTests COMMAND ${PROJECT_NAME}_UnitTests)

  foreach(TEST_SRC ${TEST_SOURCES})
    get_filename_component(TEST_NAME "${TEST_SRC}" NAME_WE)
    add_executable(${TEST_NAME} ${TEST_SRC})
    target_link_libraries(${TEST_NAME}
      ${TEST_DEPS}
      ${PROJECT_NAME}
    )
    add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
  endforeach()
endif()

