find_package(osqp QUIET)

if (NOT ${osqp_FOUND})
  message(STATUS "System install of OSQP not found - getting OSQP")
  FetchContent_Declare(osqp_extern
    GIT_REPOSITORY https://github.com/osqp/osqp
    GIT_TAG v0.6.2
  )
  FetchContent_MakeAvailable(osqp_extern)
  add_library(osqp::osqp ALIAS osqp)
  add_library(osqp::osqpstatic ALIAS osqpstatic)
  # to use osqp::osqpstatic you must also link ${CMAKE_DL_LIBS}
endif()
