find_package(osqp QUIET)

if(NOT ${osqp_FOUND})
  message(STATUS "System install of OSQP not found - using FetchContent")

  set(OSQP_FIND_VERSION 0.6.2)
  FetchContent_Declare(osqp_extern
    GIT_REPOSITORY https://github.com/osqp/osqp
    GIT_TAG v${OSQP_FIND_VERSION}
  )
  FetchContent_MakeAvailable(osqp_extern)
  add_library(osqp::osqp ALIAS osqp)
  add_library(osqp::osqpstatic ALIAS osqpstatic)
  # to use osqp::osqpstatic you must also link ${CMAKE_DL_LIBS}
  target_link_libraries(osqpstatic PUBLIC ${CMAKE_DL_LIBS})

  message(STATUS "OSQP version: ${OSQP_FIND_VERSION}")
else()
  message(STATUS "OSQP version: ${osqp_VERSION}")
endif()
