set(PYBIND11_PYTHON_VERSION 3 CACHE STRING "")
find_package(pybind11 ${PYBIND11_VERSION} QUIET) # known to work with versions 2.4.3 - 2.9.2

if (${pybind11_FOUND})
  message(STATUS "Found system install of Pybind11")
  message(STATUS "Pybind11 version: ${pybind11_VERSION}")
  if (NOT ${pybind11_VERSION} VERSION_GREATER_EQUAL "2.4.3")
    message(STATUS "Warning: lowest tested version of Pybind11 is 2.4.3")
  endif()
else ()
  message(STATUS "System install of Pybind11 not found - using FetchContent")
  FetchContent_Declare(pybind11_extern
    GIT_REPOSITORY https://github.com/pybind/pybind11
    GIT_TAG v${PYBIND11_VERSION}
  )
  FetchContent_MakeAvailable(pybind11_extern)
  # message(STATUS "Pybind11 version: ${PYBIND11_VERSION}") # should match GIT_TAG above
endif()
