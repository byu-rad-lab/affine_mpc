# DAQP, the optional dense QP solver (AFFINE_MPC_WITH_DAQP). Defines
# affine_mpc_daqp_link (the target affine_mpc links privately) and
# AFFINE_MPC_DAQP_FOUND_PACKAGE (whether it is an installed DAQP package).
#
# An installed DAQP (find_package(daqp), version 0.10.3 or newer) is used if
# found; otherwise DAQP's sources are fetched and built into a static,
# position-independent library of this project (affine_mpc_daqp), which a
# shared affine_mpc absorbs and a static one installs next to itself.

set(affine_mpc_daqp_version "0.10.3")

find_package(daqp CONFIG QUIET)
if(daqp_FOUND AND TARGET daqp)
  # DAQP installs its headers in include/daqp, while its target's include
  # directory is include
  find_path(AFFINE_MPC_DAQP_INCLUDE_DIR daqp/api.h
    HINTS "${daqp_DIR}/../include" "${daqp_DIR}/../../../include"
  )
  if(NOT AFFINE_MPC_DAQP_INCLUDE_DIR)
    message(FATAL_ERROR "Found DAQP at ${daqp_DIR} but not its headers "
      "(daqp/api.h); set AFFINE_MPC_DAQP_INCLUDE_DIR.")
  endif()
  set(affine_mpc_daqp_link daqp)
  set(AFFINE_MPC_DAQP_FOUND_PACKAGE TRUE)
  message(STATUS "Found DAQP at ${daqp_DIR}")
  return()
endif()

set(AFFINE_MPC_DAQP_FOUND_PACKAGE FALSE)
message(STATUS "Fetching DAQP ${affine_mpc_daqp_version}")
include(FetchContent)
# Only the sources are used: SOURCE_SUBDIR names a directory without a
# CMakeLists.txt, so DAQP's own project (which builds a static library
# linked to its shared one) is not added.
FetchContent_Declare(daqp_extern
  GIT_REPOSITORY https://github.com/darnstrom/daqp.git
  GIT_TAG "v${affine_mpc_daqp_version}"
  SOURCE_SUBDIR affine_mpc_sources_only
)
FetchContent_MakeAvailable(daqp_extern)

set(affine_mpc_daqp_sources)
foreach(file IN ITEMS api avi auxiliary bnb daqp daqp_prox eq_elim
                      factorization hierarchical utils)
  list(APPEND affine_mpc_daqp_sources "${daqp_extern_SOURCE_DIR}/src/${file}.c")
endforeach()

# The headers as daqp/<header>, matching an installed DAQP
file(GLOB affine_mpc_daqp_headers "${daqp_extern_SOURCE_DIR}/include/*.h")
file(COPY ${affine_mpc_daqp_headers}
  DESTINATION "${CMAKE_CURRENT_BINARY_DIR}/daqp_include/daqp"
)
set(AFFINE_MPC_DAQP_INCLUDE_DIR "${CMAKE_CURRENT_BINARY_DIR}/daqp_include")

add_library(affine_mpc_daqp STATIC ${affine_mpc_daqp_sources})
set_target_properties(affine_mpc_daqp PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  C_VISIBILITY_PRESET hidden
)
target_include_directories(affine_mpc_daqp PRIVATE
  "${daqp_extern_SOURCE_DIR}/include"
)
# DAQP's defaults: PROFILING gives the solve time and the time limit
target_compile_definitions(affine_mpc_daqp PRIVATE PROFILING)
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
  # DAQP's FASTER_MATH default
  target_compile_options(affine_mpc_daqp PRIVATE
    -fassociative-math -fno-signed-zeros -fno-trapping-math
  )
endif()
if(UNIX)
  target_link_libraries(affine_mpc_daqp PRIVATE m)
endif()
set(affine_mpc_daqp_link affine_mpc_daqp)
