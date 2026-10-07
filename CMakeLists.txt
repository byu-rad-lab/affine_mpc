cmake_minimum_required(VERSION 3.18)
project(affine_mpc
  LANGUAGES CXX C
  VERSION 0.2.0
)

# Default to an optimized build. Single-config generators (Makefiles, Ninja)
# otherwise build without optimization when no build type is given, and
# `cmake --build --config Release` only applies to multi-config generators.
# Only set when this is the top-level project so parent projects keep control.
get_property(AFFINE_MPC_MULTI_CONFIG GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG)
if(CMAKE_SOURCE_DIR STREQUAL PROJECT_SOURCE_DIR AND NOT AFFINE_MPC_MULTI_CONFIG)
  if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING
      "Build type (Debug, Release, RelWithDebInfo, MinSizeRel)" FORCE)
    message(STATUS "No build type given; defaulting to Release")
  endif()
  # dropdown in cmake-gui and ccmake
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
    Debug Release RelWithDebInfo MinSizeRel)
endif()

# --------------------------------------------------
# Standard includes
# --------------------------------------------------
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

# --------------------------------------------------
# Options
# --------------------------------------------------
option(AFFINE_MPC_BUILD_TESTS "Build unit tests" OFF)
option(AFFINE_MPC_BUILD_EXAMPLES "Build examples" OFF)

# Internal flag for building python bindings
if(NOT DEFINED AFFINE_MPC_BUILD_BINDINGS)
  set(AFFINE_MPC_BUILD_BINDINGS OFF)
endif()

# --------------------------------------------------
# Dependencies
# --------------------------------------------------
include(cmake/extern_eigen.cmake)
include(cmake/extern_osqp.cmake)
find_package(ZLIB QUIET)

# --------------------------------------------------
# Main library
# --------------------------------------------------
add_library(${PROJECT_NAME}
  src/options.cpp
  src/parameterization.cpp
  src/solve_status.cpp
  src/osqp_solver.cpp
  src/mpc_base.cpp
  src/condensed_mpc.cpp
  src/sparse_mpc.cpp
  src/mpc_logger.cpp
  src/npy_writer.cpp
  src/npz_writer.cpp
)

# Namespaced alias
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

# Library properties
set_target_properties(${PROJECT_NAME} PROPERTIES
  POSITION_INDEPENDENT_CODE ON
)
if(NOT AFFINE_MPC_BUILD_BINDINGS)
  set_target_properties(${PROJECT_NAME} PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
  )
endif()

# Include directories
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_include_directories(${PROJECT_NAME} PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# Link dependencies
target_link_libraries(${PROJECT_NAME}
  PUBLIC Eigen3::Eigen
  PUBLIC osqp::osqp
)

if(TARGET ZLIB::ZLIB)
  target_link_libraries(${PROJECT_NAME} PRIVATE ZLIB::ZLIB)
  target_compile_definitions(${PROJECT_NAME} PRIVATE AFFINE_MPC_HAS_ZLIB=1)
else()
  target_compile_definitions(${PROJECT_NAME} PRIVATE AFFINE_MPC_HAS_ZLIB=0)
endif()

# C++ standard
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

# --------------------------------------------------
# Examples
# --------------------------------------------------
if(AFFINE_MPC_BUILD_EXAMPLES)
  add_executable(example_sim examples/sim.cpp)
  target_link_libraries(example_sim PRIVATE ${PROJECT_NAME}::${PROJECT_NAME})
  # the library is C++17; the example uses C++20 designated initializers
  target_compile_features(example_sim PRIVATE cxx_std_20)
endif()

# --------------------------------------------------
# Python Bindings
# --------------------------------------------------
if(AFFINE_MPC_BUILD_BINDINGS)
  if(AFFINE_MPC_BUILD_TESTS)
    set(AFFINE_MPC_PY_ADD_TESTS ON)
  endif()

  add_subdirectory(python)
endif()

# --------------------------------------------------
# Tests
# --------------------------------------------------
if(AFFINE_MPC_BUILD_TESTS)
  enable_testing()
  include(GoogleTest)
  include(cmake/extern_gtest.cmake)
  find_package(Python COMPONENTS Interpreter QUIET)

  set(TEST_SOURCES
    test/affine_mpc/osqp_solver_tests.cpp
    test/affine_mpc/parameterization_tests.cpp
    test/affine_mpc/mpc_base_tests.cpp
    test/affine_mpc/condensed_mpc_tests.cpp
    test/affine_mpc/sparse_mpc_tests.cpp
    test/affine_mpc/consistency_tests.cpp
    test/affine_mpc/npz_writer_tests.cpp
    test/affine_mpc/mpc_logger_tests.cpp
  )

  add_executable(np_writer_fixture test/affine_mpc/np_writer_fixture.cpp)
  set_target_properties(np_writer_fixture PROPERTIES FOLDER tests)
  target_link_libraries(np_writer_fixture
    PRIVATE ${PROJECT_NAME}::${PROJECT_NAME}
  )
  target_include_directories(np_writer_fixture
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src
  )

  if(Python_Interpreter_FOUND)
    execute_process(
      COMMAND "${Python_EXECUTABLE}" -m pytest --version
      RESULT_VARIABLE pytest_found
      OUTPUT_QUIET
      ERROR_QUIET
    )
    if(pytest_found EQUAL 0)
      add_test(
        NAME python_npz_tests
        COMMAND "${Python_EXECUTABLE}" -m pytest
          "${CMAKE_CURRENT_SOURCE_DIR}/test/python_npz"
      )
      set(python_npz_tests_env
        "PYTHONPATH=${CMAKE_BINARY_DIR}"
        "AFFINE_MPC_NP_WRITER_FIXTURE=${CMAKE_BINARY_DIR}/np_writer_fixture"
        "AFFINE_MPC_HAS_ZLIB=$<IF:$<BOOL:${ZLIB_FOUND}>,1,0>"
      )
      set_tests_properties(python_npz_tests PROPERTIES
        ENVIRONMENT "${python_npz_tests_env}"
      )
    else()
      message(FATAL_ERROR "pytest is required to register Python NPZ tests")
    endif()
  else()
    message(FATAL_ERROR
      "Python interpreter is required to register Python NPZ tests"
    )
  endif()

  foreach(TEST_SRC IN LISTS TEST_SOURCES)
    get_filename_component(TEST_NAME "${TEST_SRC}" NAME_WE)
    add_executable(${TEST_NAME} ${TEST_SRC})
    set_target_properties(${TEST_NAME} PROPERTIES FOLDER tests)
    # the library is C++17; the tests use C++20 designated initializers
    target_compile_features(${TEST_NAME} PRIVATE cxx_std_20)
    target_link_libraries(${TEST_NAME} PRIVATE
      GTest::gtest_main
      ${PROJECT_NAME}::${PROJECT_NAME}
    )
    if(TEST_NAME STREQUAL "npz_writer_tests")
      target_include_directories(${TEST_NAME}
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src
      )
    endif()
    gtest_discover_tests(${TEST_NAME})
  endforeach()
endif()

# --------------------------------------------------
# Installation
# --------------------------------------------------
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  COMPONENT core
)

install(DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  COMPONENT core
)

set(config_package_location ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME})

install(EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION ${config_package_location}
  COMPONENT core
)

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)

configure_package_config_file(
  cmake/${PROJECT_NAME}Config.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  INSTALL_DESTINATION ${config_package_location}
)

install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  DESTINATION ${config_package_location}
  COMPONENT core
)
