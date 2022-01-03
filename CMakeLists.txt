cmake_minimum_required(VERSION 3.0.2)
project(osqp_mpc)

find_package(osqp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GTest)

add_library(${PROJECT_NAME} STATIC
  src/mpc_base.cpp
  src/osqp_solver.cpp
  src/implicit_mpc.cpp
  )
target_include_directories(${PROJECT_NAME} PUBLIC include)
target_link_libraries(${PROJECT_NAME} PUBLIC
  Eigen3::Eigen
  osqp::osqpstatic
  ${CMAKE_DL_LIBS} # for osqp to work
  )

if (${GTEST_FOUND})
  add_executable(${PROJECT_NAME}_UnitTests
    test/mpc_base_tests.cpp
    test/osqp_solver_tests.cpp
    test/implicit_mpc_tests.cpp
    )
  target_link_libraries(${PROJECT_NAME}_UnitTests
    gtest
    gtest_main
    pthread
    ${PROJECT_NAME}
    )
endif()
