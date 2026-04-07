find_package(Eigen3 3.4...5 QUIET NO_MODULE)

# 3.4.0 (used by Ubuntu 24.04) may not satisfy the version-range query, so
# explicitly retry with the lower bound.
if(NOT TARGET Eigen3::Eigen)
  find_package(Eigen3 3.4 QUIET)
endif()

if(NOT TARGET Eigen3::Eigen)
  message(STATUS
    "System install of Eigen not found; using FetchContent"
  )

  set(Eigen_FIND_VERSION 5.0.1)
  FetchContent_Declare(eigen3_extern
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG ${Eigen_FIND_VERSION}
  )

  # This intentionally uses FetchContent_Populate() as a fetch-only fallback for
  # Eigen. Then Eigen3::Eigen is manually defined as an interface target, rather
  # than relying on FetchContent_MakeAvailable() to integrate Eigen as a
  # subproject. This may need to be revisited if FetchContent_Populate() is
  # removed in a future CMake release.
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
