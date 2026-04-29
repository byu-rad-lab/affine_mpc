# Test-only helper for loading NPZ files with cnpy without importing cnpy's
# own install rules into this project.

find_package(ZLIB REQUIRED)

include(FetchContent)

FetchContent_Declare(cnpy_extern
  GIT_REPOSITORY https://github.com/rogersce/cnpy.git
  GIT_TAG 4e8810b
)

FetchContent_GetProperties(cnpy_extern)
if(NOT cnpy_extern_POPULATED)
  if(POLICY CMP0169)
    cmake_policy(PUSH)
    cmake_policy(SET CMP0169 OLD)
  endif()
  FetchContent_Populate(cnpy_extern)
  if(POLICY CMP0169)
    cmake_policy(POP)
  endif()
endif()

if(NOT TARGET cnpy_test)
  add_library(cnpy_test STATIC
    ${cnpy_extern_SOURCE_DIR}/cnpy.cpp
  )
  target_include_directories(cnpy_test PUBLIC ${cnpy_extern_SOURCE_DIR})
  target_link_libraries(cnpy_test PUBLIC ZLIB::ZLIB)
  set_target_properties(cnpy_test PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()

if(NOT TARGET cnpy::cnpy)
  add_library(cnpy::cnpy ALIAS cnpy_test)
endif()
