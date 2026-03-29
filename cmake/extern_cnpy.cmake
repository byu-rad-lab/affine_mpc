include(FetchContent)

# cnpy requires zlib
find_package(ZLIB REQUIRED)

# Force cnpy to build statically, even if the parent project uses BUILD_SHARED_LIBS
set(ENABLE_STATIC ON CACHE BOOL "Enable cnpy static library" FORCE)

FetchContent_Declare(cnpy_extern
  GIT_REPOSITORY https://github.com/rogersce/cnpy.git
  GIT_TAG 4e8810b
)

FetchContent_MakeAvailable(cnpy_extern)
FetchContent_GetProperties(cnpy_extern)

# Verify the static target was created
if(TARGET cnpy-static)
  # CRITICAL: Must be compiled with -fPIC to be linked into our shared library
  set_target_properties(cnpy-static PROPERTIES POSITION_INDEPENDENT_CODE ON)
  
  # Ensure the target propagates the include path
  target_include_directories(cnpy-static INTERFACE ${cnpy_extern_SOURCE_DIR})
  
  # Link ZLIB. We use PUBLIC so affine_mpc transitively gets the ZLIB link 
  # needed to resolve the static symbols inside cnpy-static.
  target_link_libraries(cnpy-static PUBLIC ZLIB::ZLIB)
  
  # Create our standard namespaced alias
  if(NOT TARGET cnpy::cnpy)
    add_library(cnpy::cnpy ALIAS cnpy-static)
  endif()
else()
  message(FATAL_ERROR "cnpy-static target not found!")
endif()
