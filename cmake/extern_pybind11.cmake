set(Python3_FIND_STRATEGY LOCATION)
find_package(Python3 COMPONENTS Interpreter Development.Module REQUIRED)

# Use FindPython mode so pybind11 integrates with the Python3 package we found.
set(PYBIND11_FINDPYTHON ON)
find_package(pybind11 CONFIG QUIET)

if(NOT TARGET pybind11::module)
  message(STATUS "System install of Pybind11 not found - using FetchContent")
  message(STATUS
    "pybind11 can be installed system-wide via `pip install pybind11[global]`"
  )

  include(FetchContent)

  set(pybind11_fetch_version "2.11.1") # Recommended stable version
  FetchContent_Declare(pybind11_extern
    GIT_REPOSITORY https://github.com/pybind/pybind11
    GIT_TAG v${pybind11_fetch_version}
  )
  FetchContent_MakeAvailable(pybind11_extern)
  message(STATUS "pybind11 version: ${pybind11_fetch_version}")
else()
  if(pybind11_VERSION VERSION_LESS "2.4.3")
    message(STATUS "Pybind11 version: ${pybind11_VERSION}")
    message(WARNING "Lowest tested version of Pybind11 is 2.4.3")
  endif()
endif()

if(NOT TARGET pybind11::module)
  message(FATAL_ERROR "Pybind11 target is missing!")
endif()
