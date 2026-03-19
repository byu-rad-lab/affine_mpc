# cnpy requires zlib
find_package(ZLIB REQUIRED)

FetchContent_Declare(cnpy_extern
  GIT_REPOSITORY https://github.com/rogersce/cnpy.git
  GIT_TAG master
)

FetchContent_MakeAvailable(cnpy_extern)

# cnpy's CMakeLists.txt is old and doesn't set target_include_directories,
# so we have to do it for it so the target propagates the include path.
if(TARGET cnpy)
  # Use FetchContent properties to get the source dir
  FetchContent_GetProperties(cnpy_extern)
  target_include_directories(cnpy INTERFACE ${cnpy_extern_SOURCE_DIR})
endif()

# cnpy doesn't define a namespaced target, let's create one for consistency
if(NOT TARGET cnpy::cnpy)
  if(TARGET cnpy)
    add_library(cnpy::cnpy ALIAS cnpy)
  endif()
endif()
