# OSQP, a private dependency: no installed affine_mpc header includes OSQP's,
# and consumers never link OSQP themselves. Defines affine_mpc_osqp_link (the
# target affine_mpc links privately), affine_mpc_osqp_target (OSQP's own
# target, which gives tests of the internal headers OSQP's headers and
# functions), and AFFINE_MPC_OSQP_FOUND_PACKAGE (whether OSQP is an installed
# package or a target defined by a parent project, rather than fetched).
#
# Shared builds use an installed OSQP if found. Otherwise OSQP's sources are
# fetched and built into a static, position-independent library with hidden
# symbols, which a shared affine_mpc absorbs and a static one installs next to
# itself under this project's name (affine_mpc_osqp).

if(BUILD_SHARED_LIBS AND NOT TARGET osqp::osqp)
  # Only look for a system OSQP when building shared libraries. A static
  # affine_mpc bundles OSQP into what links it, including the Python module
  # (static is the default, and what pip and the wheels build), so static
  # builds fetch OSQP: a system archive may not be position-independent, may
  # be built with other options (e.g. float, which affine_mpc rejects), or
  # may be unusable (Arch's libosqpstatic.a has only slim LTO objects).
  find_package(osqp 1.0.0 CONFIG QUIET)

  # This is a temporary patch to fix the Arch Linux osqp package, which currently
  # sets the version to 0.0.0 even though it builds 1.0.0
  cmake_host_system_information(RESULT is_arch QUERY DISTRIB_ID)
  if(NOT osqp_FOUND AND is_arch STREQUAL "arch")
    find_package(osqp 0.0.0 EXACT CONFIG QUIET)
    set(osqp_VERSION "1.0.0")
  endif()
endif()

if(TARGET osqp::osqp)
  set(AFFINE_MPC_OSQP_FOUND_PACKAGE TRUE)
  set(affine_mpc_osqp_link osqp::osqp)
  set(affine_mpc_osqp_target osqp::osqp)
  message(STATUS "Found OSQP version ${osqp_VERSION}")
  return()
endif()

set(AFFINE_MPC_OSQP_FOUND_PACKAGE FALSE)
if(BUILD_SHARED_LIBS)
  message(STATUS "System install of OSQP not found - using FetchContent")
else()
  message(STATUS "Fetching OSQP")
endif()

include(FetchContent)

set(osqp_find_version "1.0.0") # 0.6.3 no longer supported
FetchContent_Declare(osqp_extern
  GIT_REPOSITORY https://github.com/osqp/osqp
  GIT_TAG "v${osqp_find_version}"
)

# OSQP does not use the standard BUILD_SHARED_LIBS flag; only its static
# library is built, in every configuration
set(OSQP_BUILD_STATIC_LIB ON CACHE BOOL "Enable OSQP static library" FORCE)
set(OSQP_BUILD_SHARED_LIB OFF CACHE BOOL "Disable OSQP shared library" FORCE)
## 1.0.0 flags
# OSQP's version metadata (its CMake package version and version.h) is only
# set when OSQP_VERSION is given, and is 0.0.0 otherwise
set(OSQP_VERSION "${osqp_find_version}" CACHE STRING "" FORCE)
set(OSQP_BUILD_DEMO_EXE    OFF CACHE BOOL "" FORCE)
set(OSQP_BUILD_UNITTESTS   OFF CACHE BOOL "" FORCE)
set(OSQP_COVERAGE_CHECK    OFF CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(osqp_extern)

if(NOT TARGET osqpstatic)
  message(FATAL_ERROR "Unable to properly fetch OSQP.")
endif()

# OSQP's own install rules are skipped: nothing of OSQP is installed for a
# shared affine_mpc, and a static one installs the archive under its own name
# (see the install rules in CMakeLists.txt), since an osqp package installed
# with it could replace, or be replaced by, another OSQP in the same prefix.
set_property(DIRECTORY "${osqp_extern_SOURCE_DIR}"
  PROPERTY EXCLUDE_FROM_ALL TRUE
)
# Position-independent, so a shared affine_mpc (or the Python module) can
# absorb it, and with hidden symbols, so it does not export OSQP's API
foreach(target IN ITEMS OSQPLIB qdldlobject osqpstatic)
  if(TARGET ${target})
    set_target_properties(${target} PROPERTIES
      POSITION_INDEPENDENT_CODE ON
      C_VISIBILITY_PRESET hidden
    )
  endif()
endforeach()

set(affine_mpc_osqp_target osqpstatic)
if(BUILD_SHARED_LIBS)
  set(affine_mpc_osqp_link osqpstatic)
else()
  # A static affine_mpc passes the archive on to its consumers (link only), so
  # it is installed as affine_mpc_osqp and found through this target
  set(affine_mpc_osqp_file
    "${CMAKE_STATIC_LIBRARY_PREFIX}affine_mpc_osqp${CMAKE_STATIC_LIBRARY_SUFFIX}"
  )
  add_library(affine_mpc_osqp INTERFACE)
  target_link_libraries(affine_mpc_osqp INTERFACE
    $<BUILD_INTERFACE:osqpstatic>
    "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_LIBDIR}/${affine_mpc_osqp_file}>"
  )
  get_target_property(affine_mpc_osqp_links osqpstatic INTERFACE_LINK_LIBRARIES)
  if(affine_mpc_osqp_links)
    target_link_libraries(affine_mpc_osqp INTERFACE
      "$<INSTALL_INTERFACE:${affine_mpc_osqp_links}>"
    )
  endif()
  set(affine_mpc_osqp_link affine_mpc_osqp)
endif()
message(STATUS "Fetched OSQP version ${osqp_find_version}")
