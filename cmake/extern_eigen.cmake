find_package(Eigen3 3.4...5 QUIET NO_MODULE)

# 3.4.0 (used by Ubuntu 24.04) may not satisfy the version-range query, so
# explicitly retry with the lower bound.
if(NOT TARGET Eigen3::Eigen)
  find_package(Eigen3 3.4 QUIET NO_MODULE)
endif()

if(NOT TARGET Eigen3::Eigen)
  if(NOT AFFINE_MPC_BINDINGS)
    # System install of Eigen is required for normal builds
    message(FATAL_ERROR
      "Eigen3 not found. Please install Eigen (>= 3.4) on your system "
      "before building affine_mpc."
    )
  endif()

  # Everything below is intended as a fallback solely for building bindings on
  # Github servers, which may not have Eigen installed. For normal builds,
  # using the FetchContent target may cause issues downstream.
  message(STATUS
    "System install of Eigen3 not found; using FetchContent fallback for bindings build")

  set(Eigen_FIND_VERSION 5.0.1)
  FetchContent_Declare(eigen3_extern
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG ${Eigen_FIND_VERSION}
  )

  FetchContent_GetProperties(eigen3_extern)
  if(NOT eigen3_extern_POPULATED)
    FetchContent_Populate(eigen3_extern)
  endif()

  if(NOT TARGET Eigen3::Eigen)
    add_library(Eigen3::Eigen INTERFACE IMPORTED GLOBAL)
    set_target_properties(Eigen3::Eigen PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES "${eigen3_extern_SOURCE_DIR}"
    )
  endif()

  set(Eigen3_FOUND TRUE)
  set(Eigen3_VERSION ${Eigen_FIND_VERSION})
endif()

message(STATUS "Eigen3 version: ${Eigen3_VERSION}")
