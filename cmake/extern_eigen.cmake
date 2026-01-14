find_package(Eigen3 3.4...5 QUIET NO_MODULE)

if(NOT TARGET Eigen3::Eigen)
  message(STATUS "System install of Eigen3 not found - using FetchContent")

  set(Eigen_FIND_VERSION 5.0.1)
  FetchContent_Declare(eigen3_extern
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG ${Eigen_FIND_VERSION}
    EXCLUDE_FROM_ALL
  )
  FetchContent_MakeAvailable(eigen3_extern)

  message(STATUS "Eigen3 version: ${Eigen_FIND_VERSION}")
else()
  message(STATUS "Eigen3 version: ${Eigen3_VERSION}")
endif()
