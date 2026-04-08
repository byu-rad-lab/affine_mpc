find_package(GTest 1.11 QUIET)

if(NOT TARGET GTest::gtest)
  message(STATUS "System install of GTest not found - using FetchContent")

  include(FetchContent)

  set(gtest_fetch_version 1.17.0)
  FetchContent_Declare(gtest_extern
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v${gtest_fetch_version}
  )
  FetchContent_MakeAvailable(gtest_extern)

  message(STATUS "GTest version: ${gtest_fetch_version}")
else()
  message(STATUS "GTest version: ${GTest_VERSION}")
endif()
