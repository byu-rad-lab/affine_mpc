cmake_minimum_required(VERSION 3.18)
project(affine_mpc
  LANGUAGES CXX C
  VERSION 1.0.0
)

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
include(cmake/extern_cnpy.cmake)

# --------------------------------------------------
# Main library
# --------------------------------------------------
add_library(${PROJECT_NAME}
  src/mpc_base.cpp
  src/osqp_solver.cpp
  src/condensed_mpc.cpp
  src/sparse_mpc.cpp
  src/parameterization.cpp
  src/mpc_logger.cpp
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

# Link dependencies
target_link_libraries(${PROJECT_NAME}
  PUBLIC Eigen3::Eigen
  PUBLIC osqp::osqp
  PRIVATE $<BUILD_INTERFACE:cnpy::cnpy>
)

# C++ standard
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

# --------------------------------------------------
# Examples
# --------------------------------------------------
if(AFFINE_MPC_BUILD_EXAMPLES)
  add_executable(example_sim examples/sim.cpp)
  target_link_libraries(example_sim PRIVATE ${PROJECT_NAME}::${PROJECT_NAME})
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

  set(TEST_SOURCES
    test/affine_mpc/osqp_solver_tests.cpp
    test/affine_mpc/parameterization_tests.cpp
    test/affine_mpc/mpc_base_tests.cpp
    test/affine_mpc/condensed_mpc_tests.cpp
    test/affine_mpc/sparse_mpc_tests.cpp
    test/affine_mpc/consistency_tests.cpp
    test/affine_mpc/mpc_logger_tests.cpp
  )

  foreach(TEST_SRC IN LISTS TEST_SOURCES)
    get_filename_component(TEST_NAME "${TEST_SRC}" NAME_WE)
    add_executable(${TEST_NAME} ${TEST_SRC})
    set_target_properties(${TEST_NAME} PROPERTIES FOLDER tests)
    target_link_libraries(${TEST_NAME} PRIVATE
      GTest::gtest_main
      ${PROJECT_NAME}::${PROJECT_NAME}
      cnpy::cnpy
    )
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
)

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

set(config_package_location ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME})

install(EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION ${config_package_location}
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
)
