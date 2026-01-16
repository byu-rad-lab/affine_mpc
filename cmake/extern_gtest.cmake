find_package(GTest 1.11 QUIET)

if(NOT TARGET GTest::gtest)
  message(STATUS "System install of GTest not found - using FetchContent")

  set(GTest_FIND_VERSION 1.17.0)
  FetchContent_Declare(gtest_extern
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v${GTest_FIND_VERSION}
    EXCLUDE_FROM_ALL
  )
  FetchContent_MakeAvailable(gtest_extern)

  message(STATUS "GTest version: ${GTest_FIND_VERSION}")
else()
  message(STATUS "GTest version: ${GTest_VERSION}")
endif()
