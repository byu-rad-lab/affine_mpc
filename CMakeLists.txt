cmake_minimum_required(VERSION 3.28)
project(affine_mpc
  LANGUAGES CXX
  VERSION 1.0.0
)
# OSQP 0.6.x has minimum version of 3.2, which CMake no longer supports
# Can remove after migrating to OSQP >1.0
set(CMAKE_POLICY_VERSION_MINIMUM 3.10)

# --------------------------------------------------
# Options
# --------------------------------------------------
option(${PROJECT_NAME}_BUILD_TESTS "Build unit tests" OFF)
option(${PROJECT_NAME}_BUILD_EXAMPLE "Build examples" OFF)
option(${PROJECT_NAME}_SHARED_LIBS "Build shared libraries" ON)
option(DISABLE_SHARED_VERSIONS "Disable shared library versions" OFF)

if(IS_TOP_LEVEL_PROJECT)
  set(BUILD_SHARED_LIBS
    ${${PROJECT_NAME}_SHARED_LIBS}
    CACHE BOOL "Build shared libraries" FORCE
  )
endif()

# --------------------------------------------------
# Compiler and visibility settings
# --------------------------------------------------
# if(BUILD_SHARED_LIBS)
#   set(CMAKE_CXX_VISIBILITY_PRESET hidden CACHE STRING
#     "Set the default C++ symbol visibility to hidden"
#   )
#   set(CMAKE_VISIBILITY_INLINES_HIDDEN YES CACHE BOOL
#     "Set the default C++ inline function visibility to hidden"
#   )
# endif()

# --------------------------------------------------
# Dependencies
# --------------------------------------------------
include(FetchContent)
include(cmake/extern_eigen.cmake)
include(cmake/extern_osqp.cmake)

# --------------------------------------------------
# Main library
# --------------------------------------------------
add_library(${PROJECT_NAME}
  src/mpc_base.cpp
  src/osqp_solver.cpp
  src/implicit_mpc.cpp
  src/bspline_mpc.cpp
  src/mpc_logger.cpp
)

# Namespaced alias
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

# Library properties
set_target_properties(${PROJECT_NAME} PROPERTIES
  POSITION_INDEPENDENT_CODE ON
)
if(NOT ${DISABLE_SHARED_VERSIONS})
  set_target_properties(${PROJECT_NAME} PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
  )
endif()

# Include directories
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link dependencies
target_link_libraries(${PROJECT_NAME} PUBLIC
  Eigen3::Eigen
  osqp::osqp
)

# C++ standard
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

# --------------------------------------------------
# Examples
# --------------------------------------------------
if(${PROJECT_NAME}_BUILD_EXAMPLE)
  add_executable(example_sim examples/sim.cpp)
  target_link_libraries(example_sim PRIVATE ${PROJECT_NAME}::${PROJECT_NAME})
endif()

# --------------------------------------------------
# Tests
# --------------------------------------------------
if(${PROJECT_NAME}_BUILD_TESTS)
  enable_testing()
  include(GoogleTest)
  include(cmake/extern_gtest.cmake)

  set(TEST_SOURCES
    test/affine_mpc/mpc_base_tests.cpp
    test/affine_mpc/osqp_solver_tests.cpp
    test/affine_mpc/implicit_mpc_tests.cpp
    test/affine_mpc/bspline_mpc_tests.cpp
    # test/affine_mpc/mpc_logger_tests.cpp
  )

  foreach(TEST_SRC IN LISTS TEST_SOURCES)
    get_filename_component(TEST_NAME "${TEST_SRC}" NAME_WE)
    add_executable(${TEST_NAME} ${TEST_SRC})
    target_link_libraries(${TEST_NAME} PRIVATE
      GTest::gtest_main
      ${PROJECT_NAME}::${PROJECT_NAME}
    )
    gtest_discover_tests(${TEST_NAME})
  endforeach()
endif()

# --------------------------------------------------
# Installation
# --------------------------------------------------
# install(TARGETS ${PROJECT_NAME}
#     EXPORT ${PROJECT_NAME}Targets
#     ARCHIVE DESTINATION lib
#     LIBRARY DESTINATION lib
#     RUNTIME DESTINATION bin
#     PUBLIC_HEADER DESTINATION include
# )
#
# install(DIRECTORY include/ DESTINATION include)
#
# install(EXPORT ${PROJECT_NAME}Targets
#     FILE ${PROJECT_NAME}Targets.cmake
#     NAMESPACE ${PROJECT_NAME}::
#     DESTINATION lib/cmake/${PROJECT_NAME}
# )
#
# include(CMakePackageConfigHelpers)
# write_basic_package_version_file(
#     "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
#     VERSION ${PROJECT_VERSION}
#     COMPATIBILITY AnyNewerVersion
# )
#
# configure_file(cmake/${PROJECT_NAME}Config.cmake.in
#     "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
#     @ONLY
# )
#
# install(FILES
#     "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
#     "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
#     DESTINATION lib/cmake/${PROJECT_NAME}
# )
