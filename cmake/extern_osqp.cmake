if(BUILD_SHARED_LIBS)
  # system install osqpstatic had linking issues, so just use fetchcontent for
  # static builds to manage it here
  find_package(osqp 1.0.0 CONFIG QUIET)

  # This is a temporary patch to fix the Arch linux osqp package, which currently
  # sets the version to 0.0.0 even though it builds 1.0.0
  cmake_host_system_information(RESULT is_arch QUERY DISTRIB_ID)
  if(NOT osqp_FOUND AND is_arch STREQUAL "arch")
    find_package(osqp 0.0.0 EXACT CONFIG QUIET)
    set(osqp_VERSION "1.0.0")
  endif()
endif()

if(NOT TARGET osqp::osqp)
  if(BUILD_SHARED_LIBS)
    message(STATUS "System install of OSQP not found - using FetchContent")
  else()
    message(STATUS "Fetching OSQP")
  endif()

  set(osqp_find_version "1.0.0") # 0.6.3 no longer supported
  FetchContent_Declare(osqp_extern
    GIT_REPOSITORY https://github.com/osqp/osqp
    GIT_TAG "v${osqp_find_version}"
  )

  # Configure OSQP build
  if(BUILD_SHARED_LIBS)
    # OSQP does not use the standard BUILD_SHARED_LIBS flag. This sets the OSQP
    # flags to match BUILD_SHARED_LIBS
    set(OSQP_BUILD_STATIC_LIB OFF CACHE BOOL
      "Disable OSQP static library" FORCE
    )
    set(OSQP_BUILD_SHARED_LIB ON CACHE BOOL
      "Enable OSQP shared library" FORCE
    )
  else()
    set(OSQP_BUILD_STATIC_LIB ON CACHE BOOL
      "Enable OSQP static library" FORCE
    )
    set(OSQP_BUILD_SHARED_LIB OFF CACHE BOOL
      "Disable OSQP shared library" FORCE
    )
  endif()
  ## 1.0.0 flags
  set(OSQP_BUILD_DEMO_EXE    OFF CACHE BOOL "" FORCE)
  set(OSQP_BUILD_UNITTESTS   OFF CACHE BOOL "" FORCE)
  set(OSQP_COVERAGE_CHECK    OFF CACHE BOOL "" FORCE)
  ## 0.6.3 flags
  # set(OSQP_ENABLE_UNIT_TESTS OFF CACHE BOOL "" FORCE)
  # set(OSQP_ENABLE_MATLAB     OFF CACHE BOOL "" FORCE)
  # set(OSQP_ENABLE_DEMO       OFF CACHE BOOL "" FORCE)

  FetchContent_MakeAvailable(osqp_extern)

  # Normalize to osqp::osqp
  if(NOT TARGET osqp::osqp)
    if(BUILD_SHARED_LIBS AND TARGET osqp)
      add_library(osqp::osqp ALIAS osqp)
    elseif(NOT BUILD_SHARED_LIBS AND TARGET osqpstatic)
      add_library(osqp::osqp ALIAS osqpstatic)
    else()
      message(FATAL_ERROR "Unable to properly fetch OSQP.")
    endif()
    message(STATUS "Fetched OSQP version ${osqp_find_version}")
  endif()
else()
  message(STATUS "Found OSQP version ${osqp_VERSION}")
endif()
