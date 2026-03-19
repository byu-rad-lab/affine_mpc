find_package(osqp 1.0 CONFIG QUIET)

if(NOT osqp_FOUND)
  message(STATUS "System install of OSQP not found - using FetchContent")

  FetchContent_Declare(osqp_extern
    GIT_REPOSITORY https://github.com/osqp/osqp
    # GIT_TAG v0.6.3 # no longer supported
    GIT_TAG v1.0.0
  )

  # OSQP options — disable things you do not need
  if(BUILD_SHARED_LIBS)
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
  set(OSQP_ENABLE_UNIT_TESTS OFF CACHE BOOL "" FORCE)
  set(OSQP_ENABLE_MATLAB     OFF CACHE BOOL "" FORCE)
  set(OSQP_ENABLE_DEMO       OFF CACHE BOOL "" FORCE)

  FetchContent_MakeAvailable(osqp_extern)
endif()

# Normalize to osqp::osqp
if(NOT TARGET osqp::osqp)

  # prefer according to BUILD_SHARED_LIBS
  if(BUILD_SHARED_LIBS AND TARGET osqp)
    add_library(osqp::osqp ALIAS osqp)
  elseif(NOT BUILD_SHARED_LIBS AND TARGET osqpstatic)
    add_library(osqp::osqp ALIAS osqpstatic)

  # fallback: pick whichever variant exists, shared preferred
  elseif(TARGET osqp)
    add_library(osqp::osqp ALIAS osqp)
  elseif(TARGET osqpstatic)
    add_library(osqp::osqp ALIAS osqpstatic)

  # neither found
  else()
    message(FATAL_ERROR "OSQP found but no usable target exists")
  endif()
endif()
