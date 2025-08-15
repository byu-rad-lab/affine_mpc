find_package(Eigen3 3.4 QUIET)

if (NOT ${Eigen3_FOUND})
  message(STATUS "System install of Eigen not found - using FetchContent")
  FetchContent_Declare(eigen_extern
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen
    GIT_TAG 3.4.0
  )
  FetchContent_MakeAvailable(eigen_extern)
else ()
  message(STATUS "Eigen version found: ${Eigen3_VERSION}")
endif()
