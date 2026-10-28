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
option(AFFINE_MPC_WITH_DAQP
  "Build the DAQP solver (dense active set) for the condensed formulation" OFF)
include(cmake/compiler_warnings.cmake)

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
if(AFFINE_MPC_WITH_DAQP)
  include(cmake/extern_daqp.cmake)
endif()

# --------------------------------------------------
# Main library
# --------------------------------------------------
add_library(${PROJECT_NAME}
  src/options.cpp
  src/parameterization.cpp
  src/solve_status.cpp
  src/osqp_solver.cpp
  src/vector_parameterization.cpp
  src/discretizer.cpp
  src/mpc.cpp
  src/formulation.cpp
  src/qp_terms.cpp
  src/qp_builder.cpp
  src/mpc_logger.cpp
  src/npy_writer.cpp
  src/npz_writer.cpp
)
if(AFFINE_MPC_WITH_DAQP)
  target_sources(${PROJECT_NAME} PRIVATE src/daqp_solver.cpp)
  target_include_directories(${PROJECT_NAME} PRIVATE
    ${AFFINE_MPC_DAQP_INCLUDE_DIR}
  )
  target_link_libraries(${PROJECT_NAME} PRIVATE ${affine_mpc_daqp_link})
  # public, so consumers can check for DAQPSolver
  target_compile_definitions(${PROJECT_NAME} PUBLIC AFFINE_MPC_WITH_DAQP)
endif()

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
affine_mpc_enable_warnings(${PROJECT_NAME})

# --------------------------------------------------
# Examples
# --------------------------------------------------
if(AFFINE_MPC_BUILD_EXAMPLES)
  add_executable(example_sim examples/sim.cpp)
  target_link_libraries(example_sim PRIVATE ${PROJECT_NAME}::${PROJECT_NAME})
  # the library is C++17; the example uses C++20 designated initializers
  target_compile_features(example_sim PRIVATE cxx_std_20)
  affine_mpc_enable_warnings(example_sim)
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
    test/affine_mpc/qp_solver_tests.cpp
    test/affine_mpc/parameterization_tests.cpp
    test/affine_mpc/discretizer_tests.cpp
    test/affine_mpc/mpc_tests.cpp
    test/affine_mpc/condensed_mpc_tests.cpp
    test/affine_mpc/sparse_mpc_tests.cpp
    test/affine_mpc/consistency_tests.cpp
    test/affine_mpc/change_category_tests.cpp
    test/affine_mpc/npz_writer_tests.cpp
    test/affine_mpc/mpc_logger_tests.cpp
    test/affine_mpc/allocation_tests.cpp
    test/affine_mpc/input_bounds_tests.cpp
    test/affine_mpc/input_cost_tests.cpp
    test/affine_mpc/input_rate_cost_tests.cpp
    test/affine_mpc/state_bounds_tests.cpp
    test/affine_mpc/general_constraints_tests.cpp
    test/affine_mpc/qp_pattern_tests.cpp
    test/affine_mpc/model_sparsity_tests.cpp
    test/affine_mpc/utils_tests.cpp
  )
  if(AFFINE_MPC_WITH_DAQP)
    list(APPEND TEST_SOURCES test/affine_mpc/daqp_solver_tests.cpp)
  endif()

  add_executable(np_writer_fixture test/affine_mpc/np_writer_fixture.cpp)
  set_target_properties(np_writer_fixture PROPERTIES FOLDER tests)
  target_link_libraries(np_writer_fixture
    PRIVATE ${PROJECT_NAME}::${PROJECT_NAME}
  )
  target_include_directories(np_writer_fixture
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src
  )
  affine_mpc_enable_warnings(np_writer_fixture)

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
        "AFFINE_MPC_NP_WRITER_FIXTURE=$<TARGET_FILE:np_writer_fixture>"
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
    affine_mpc_enable_warnings(${TEST_NAME})
    target_link_libraries(${TEST_NAME} PRIVATE
      GTest::gtest_main
      ${PROJECT_NAME}::${PROJECT_NAME}
    )
    # tests of internal headers (src/)
    if(TEST_NAME MATCHES "^(npz_writer|mpc|condensed_mpc|general_constraints|qp_pattern|model_sparsity)_tests$")
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
get_target_property(affine_mpc_library_type ${PROJECT_NAME} TYPE)
# A static affine_mpc needs the fetched DAQP library next to it
set(affine_mpc_install_targets ${PROJECT_NAME})
if(AFFINE_MPC_WITH_DAQP AND NOT AFFINE_MPC_DAQP_FOUND_PACKAGE
   AND affine_mpc_library_type STREQUAL "STATIC_LIBRARY")
  list(APPEND affine_mpc_install_targets affine_mpc_daqp)
endif()
install(TARGETS ${affine_mpc_install_targets}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  COMPONENT core
)

if(AFFINE_MPC_WITH_DAQP)
  install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    COMPONENT core
  )
else()
  install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    COMPONENT core
    PATTERN daqp_solver.hpp EXCLUDE
  )
endif()

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

if(TARGET ZLIB::ZLIB AND affine_mpc_library_type STREQUAL "STATIC_LIBRARY")
  set(AFFINE_MPC_CONFIG_NEEDS_ZLIB TRUE)
else()
  set(AFFINE_MPC_CONFIG_NEEDS_ZLIB FALSE)
endif()
# Likewise its private link to an installed DAQP package
if(AFFINE_MPC_WITH_DAQP AND AFFINE_MPC_DAQP_FOUND_PACKAGE
   AND affine_mpc_library_type STREQUAL "STATIC_LIBRARY")
  set(AFFINE_MPC_CONFIG_NEEDS_DAQP TRUE)
else()
  set(AFFINE_MPC_CONFIG_NEEDS_DAQP FALSE)
endif()

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
